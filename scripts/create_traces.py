#!/usr/bin/env python3
"""
Post-process a ROS 2 MCAP bag: read PoseStamped local position topics and write
rolling-window traces as foxglove.SceneUpdate line strips.

The output bag is a copy of the input with two extra topics added, so the
original data is untouched and Foxglove renders the traces natively.

Usage:
    python3 create_traces.py input.mcap output.mcap \
        --trace /SQ01/mavros/local_position/pose:SQ01:0,0.8,1.0 \
        --trace /SQ02/mavros/local_position/pose:SQ02:1.0,0.5,0.1 \
        --window 10.0
"""

import argparse
import json
import sys
from collections import deque

from mcap.reader import make_reader
from mcap.writer import Writer
from mcap_ros2.decoder import DecoderFactory

# ---- foxglove.SceneUpdate schema (JSON encoding) -------------------------
SCENE_UPDATE_JSONSCHEMA = {
    "type": "object",
    "properties": {
        "deletions": {"type": "array", "items": {"type": "object"}},
        "entities": {
            "type": "array",
            "items": {
                "type": "object",
                "properties": {
                    "timestamp": {
                        "type": "object",
                        "properties": {
                            "sec": {"type": "integer"},
                            "nsec": {"type": "integer"},
                        },
                    },
                    "frame_id": {"type": "string"},
                    "id": {"type": "string"},
                    "lifetime": {
                        "type": "object",
                        "properties": {
                            "sec": {"type": "integer"},
                            "nsec": {"type": "integer"},
                        },
                    },
                    "frame_locked": {"type": "boolean"},
                    "spheres": {
                        "type": "array",
                        "items": {
                            "type": "object",
                            "properties": {
                                "pose": {"type": "object"},
                                "size": {"type": "object"},
                                "color": {"type": "object"},
                            },
                        },
                    },
                    "lines": {
                        "type": "array",
                        "items": {
                            "type": "object",
                            "properties": {
                                "type": {"type": "integer"},
                                "pose": {"type": "object"},
                                "thickness": {"type": "number"},
                                "scale_invariant": {"type": "boolean"},
                                "points": {"type": "array", "items": {"type": "object"}},
                                "color": {"type": "object"},
                                "colors": {"type": "array", "items": {"type": "object"}},
                                "indices": {"type": "array", "items": {"type": "integer"}},
                            },
                        },
                    },
                },
            },
        },
    },
}

IDENTITY_POSE = {
    "position": {"x": 0.0, "y": 0.0, "z": 0.0},
    "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
}

# foxglove.FrameTransform (JSON encoding)
FRAME_TRANSFORM_JSONSCHEMA = {
    "type": "object",
    "properties": {
        "timestamp": {
            "type": "object",
            "properties": {
                "sec": {"type": "integer"},
                "nsec": {"type": "integer"},
            },
        },
        "parent_frame_id": {"type": "string"},
        "child_frame_id": {"type": "string"},
        "translation": {
            "type": "object",
            "properties": {
                "x": {"type": "number"},
                "y": {"type": "number"},
                "z": {"type": "number"},
            },
        },
        "rotation": {
            "type": "object",
            "properties": {
                "x": {"type": "number"},
                "y": {"type": "number"},
                "z": {"type": "number"},
                "w": {"type": "number"},
            },
        },
    },
}


def build_frame_transform(parent, child, stamp_sec, stamp_nsec):
    """Identity transform: parent and child are the same frame, no offset."""
    return {
        "timestamp": {"sec": stamp_sec, "nsec": stamp_nsec},
        "parent_frame_id": parent,
        "child_frame_id": child,
        "translation": {"x": 0.0, "y": 0.0, "z": 0.0},
        "rotation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
    }


def parse_trace_spec(spec):
    """'/topic:id:r,g,b' -> (topic, entity_id, (r,g,b))."""
    parts = spec.split(":")
    if len(parts) != 3:
        raise argparse.ArgumentTypeError(
            f"--trace must be TOPIC:ID:R,G,B, got '{spec}'"
        )
    topic, entity_id, rgb = parts
    r, g, b = (float(c) for c in rgb.split(","))
    return topic, entity_id, (r, g, b)


def build_scene_update(entity_id, frame_id, stamp_sec, stamp_nsec, pts, color,
                       thickness, scale_invariant):
    return {
        "deletions": [],
        "entities": [
            {
                "timestamp": {"sec": stamp_sec, "nsec": stamp_nsec},
                "frame_id": frame_id,
                "id": entity_id,
                "lifetime": {"sec": 0, "nsec": 0},
                "frame_locked": False,
                "lines": [
                    {
                        "type": 0,  # LINE_STRIP
                        "pose": IDENTITY_POSE,
                        "thickness": thickness,
                        "scale_invariant": scale_invariant,
                        "points": [{"x": p[0], "y": p[1], "z": p[2]} for p in pts],
                        "color": {"r": color[0], "g": color[1], "b": color[2], "a": 1.0},
                        "colors": [],
                        "indices": [],
                    }
                ],
            }
        ],
    }


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("input")
    ap.add_argument("output")
    ap.add_argument(
        "--trace",
        action="append",
        type=parse_trace_spec,
        required=True,
        help="TOPIC:ENTITY_ID:R,G,B  (colors 0-1). Repeatable.",
    )
    ap.add_argument("--window", type=float, default=10.0,
                    help="Rolling buffer length in seconds (default 10).")
    ap.add_argument("--out-suffix", default="_trace",
                    help="Suffix for generated topic names (default '_trace').")
    ap.add_argument("--thickness", type=float, default=2.0)
    ap.add_argument("--world-units", action="store_true",
                    help="Thickness in world meters instead of screen pixels.")
    args = ap.parse_args()
    y_offsets = {
        "/SQ01/mavros/local_position/pose": 3.0,
        "/SQ02/mavros/local_position/pose": 6.0,
    }
    trace_topics = {t[0]: (t[1], t[2]) for t in args.trace}
    out_topic_for = {t[0]: f"{t[0]}{args.out_suffix}" for t in args.trace}
    buffers = {t[0]: deque() for t in args.trace}
    scale_invariant = not args.world_units

    with open(args.input, "rb") as f_in:
        reader = make_reader(f_in, decoder_factories=[DecoderFactory()])
        summary = reader.get_summary()

        with open(args.output, "wb") as f_out:
            writer = Writer(f_out)
            writer.start()

            # Copy original schemas and channels
            schema_id_map = {}
            channel_id_map = {}
            for schema in summary.schemas.values():
                schema_id_map[schema.id] = writer.register_schema(
                    name=schema.name, encoding=schema.encoding, data=schema.data
                )
            for channel in summary.channels.values():
                channel_id_map[channel.id] = writer.register_channel(
                    topic=channel.topic,
                    message_encoding=channel.message_encoding,
                    schema_id=schema_id_map[channel.schema_id],
                    metadata=channel.metadata,
                )

            # Register SceneUpdate output schema/channel
            scene_schema_id = writer.register_schema(
                name="foxglove.SceneUpdate",
                encoding="jsonschema",
                data=json.dumps(SCENE_UPDATE_JSONSCHEMA).encode("utf-8"),
            )
            trace_channel_id = {}
            for src_topic in trace_topics:
                trace_channel_id[src_topic] = writer.register_channel(
                    topic=out_topic_for[src_topic],
                    message_encoding="json",
                    schema_id=scene_schema_id,
                    metadata={},
                )

            n_in = 0
            n_trace = 0

            for schema, channel, message, ros_msg in reader.iter_decoded_messages():
                # Re-emit original message unchanged
                writer.add_message(
                    channel_id=channel_id_map[channel.id],
                    log_time=message.log_time,
                    publish_time=message.publish_time,
                    data=message.data,
                    sequence=message.sequence,
                )
                n_in += 1

                if channel.topic not in trace_topics:
                    continue

                entity_id, color = trace_topics[channel.topic]

                # PoseStamped stamp and frame
                stamp = ros_msg.header.stamp
                frame_id = ros_msg.header.frame_id or "world"

                # PoseStamped position
                p = ros_msg.pose.position

                t = stamp.sec + stamp.nanosec * 1e-9
                buf = buffers[channel.topic]

                # Reset buffer if time goes backwards
                if buf and t < buf[-1][0]:
                    buf.clear()

                y_offset = y_offsets.get(channel.topic, 0.0)
                buf.append((t, p.x, p.y + y_offset, p.z))

                # Keep only window-length history
                while buf and (t - buf[0][0]) > args.window:
                    buf.popleft()

                scene = build_scene_update(
                    entity_id,
                    frame_id,
                    stamp.sec,
                    stamp.nanosec,
                    [(x, y, z) for (_, x, y, z) in buf],
                    color,
                    args.thickness,
                    scale_invariant,
                )

                writer.add_message(
                    channel_id=trace_channel_id[channel.topic],
                    log_time=message.log_time,
                    publish_time=message.publish_time,
                    data=json.dumps(scene).encode("utf-8"),
                    sequence=message.sequence,
                )
                n_trace += 1

            writer.finish()

    print(f"Copied {n_in} original messages, wrote {n_trace} trace messages.")
    print("New topics:")
    for src, out in out_topic_for.items():
        print(f"  {out}   <- rolling {args.window:.0f}s of {src}")


if __name__ == "__main__":
    main()