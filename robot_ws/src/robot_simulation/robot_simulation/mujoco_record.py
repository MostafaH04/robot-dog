"""Generate compact deterministic review animations for MuJoCo models."""

import argparse
import hashlib
import json
from pathlib import Path
import struct
import zlib

import mujoco

import numpy as np

from robot_simulation.mujoco_core import (
    FOOT_NAMES,
    MODEL_VARIANTS,
    MujocoSimulator,
    trajectory_fingerprint,
)


FRAME_COUNT = 48
FRAMES_PER_SECOND = 12
FRAME_WIDTH = 360
FRAME_HEIGHT = 270
CONTACT_COLOR = (0.12, 0.85, 0.24, 1.0)
FREE_COLOR = (0.12, 0.12, 0.12, 1.0)


def _png_chunk(chunk_type, payload):
    """Encode one checksummed PNG chunk."""
    checksum = zlib.crc32(chunk_type)
    checksum = zlib.crc32(payload, checksum)
    return (
        struct.pack('>I', len(payload))
        + chunk_type
        + payload
        + struct.pack('>I', checksum & 0xffffffff)
    )


def _compressed_pixels(frame):
    """Encode one RGB frame with deterministic PNG filter selection."""
    rows = (
        b'\x00' + np.ascontiguousarray(row).tobytes()
        for row in frame
    )
    return zlib.compress(b''.join(rows), level=9)


def _write_png(path, frame):
    """Write one RGB image using only the Python standard library."""
    height, width, channels = frame.shape
    if channels != 3 or frame.dtype != np.uint8:
        raise ValueError('PNG frames must be uint8 RGB arrays')
    payload = bytearray(b'\x89PNG\r\n\x1a\n')
    payload.extend(_png_chunk(
        b'IHDR',
        struct.pack('>IIBBBBB', width, height, 8, 2, 0, 0, 0),
    ))
    payload.extend(_png_chunk(b'IDAT', _compressed_pixels(frame)))
    payload.extend(_png_chunk(b'IEND', b''))
    path.write_bytes(payload)


def _write_apng(path, frames, frames_per_second):
    """Write a looping APNG without adding encoder dependencies."""
    if not frames:
        raise ValueError('APNG requires at least one frame')
    height, width, channels = frames[0].shape
    if channels != 3 or frames[0].dtype != np.uint8:
        raise ValueError('APNG frames must be uint8 RGB arrays')

    payload = bytearray(b'\x89PNG\r\n\x1a\n')
    payload.extend(_png_chunk(
        b'IHDR',
        struct.pack('>IIBBBBB', width, height, 8, 2, 0, 0, 0),
    ))
    payload.extend(_png_chunk(b'acTL', struct.pack('>II', len(frames), 0)))

    sequence = 0
    for index, frame in enumerate(frames):
        if frame.shape != (height, width, 3) or frame.dtype != np.uint8:
            raise ValueError('all APNG frames must share uint8 RGB shape')
        frame_control = struct.pack(
            '>IIIIIHHBB',
            sequence,
            width,
            height,
            0,
            0,
            1,
            frames_per_second,
            0,
            0,
        )
        payload.extend(_png_chunk(b'fcTL', frame_control))
        sequence += 1
        pixels = _compressed_pixels(frame)
        if index == 0:
            payload.extend(_png_chunk(b'IDAT', pixels))
        else:
            payload.extend(_png_chunk(
                b'fdAT',
                struct.pack('>I', sequence) + pixels,
            ))
            sequence += 1

    payload.extend(_png_chunk(b'IEND', b''))
    path.write_bytes(payload)


def _camera():
    """Return the fixed review camera shared by both variants."""
    camera = mujoco.MjvCamera()
    mujoco.mjv_defaultCamera(camera)
    camera.lookat[:] = (0.0, 0.0, 0.12)
    camera.distance = 0.75
    camera.azimuth = 135.0
    camera.elevation = -20.0
    return camera


def _record_variant(model_variant, output_directory):
    """Render one fixed-duration settle-and-hold sequence."""
    simulator = MujocoSimulator(
        settle_steps=0,
        model_variant=model_variant,
    )
    renderer = mujoco.Renderer(
        simulator.model,
        height=FRAME_HEIGHT,
        width=FRAME_WIDTH,
    )
    camera = _camera()
    steps_per_frame = round(
        1.0 / (FRAMES_PER_SECOND * simulator.timestep),
    )
    frames = []
    samples = []
    contact_frame_counts = {foot: 0 for foot in FOOT_NAMES}
    result = simulator.observe()

    try:
        for _ in range(FRAME_COUNT):
            samples.append(result)
            for foot in FOOT_NAMES:
                contact = result.sensors.foot_contacts[foot].in_contact
                contact_frame_counts[foot] += int(contact)
                geom_id = simulator.model.geom(f'{foot}_foot_geom').id
                simulator.model.geom_rgba[geom_id] = (
                    CONTACT_COLOR if contact else FREE_COLOR
                )
            renderer.update_scene(simulator.data, camera=camera)
            frames.append(renderer.render().copy())
            for _ in range(steps_per_frame):
                result = simulator.step()
    finally:
        renderer.close()

    animation_path = output_directory / f'{model_variant}.apng'
    poster_path = output_directory / f'{model_variant}-contact.png'
    _write_apng(animation_path, frames, FRAMES_PER_SECOND)
    _write_png(poster_path, frames[-1])
    fingerprint = hashlib.sha256(
        trajectory_fingerprint(samples).tobytes(),
    ).hexdigest()
    return {
        'animation': animation_path.name,
        'contact_frame_counts': contact_frame_counts,
        'duration_seconds': FRAME_COUNT / FRAMES_PER_SECOND,
        'fingerprint_sha256': fingerprint,
        'frames': FRAME_COUNT,
        'poster': poster_path.name,
        'steps_per_frame': steps_per_frame,
    }


def _write_index(output_directory, manifest):
    """Write a self-contained artifact index for side-by-side review."""
    cards = []
    for model_variant in MODEL_VARIANTS:
        result = manifest['models'][model_variant]
        cards.append(
            '<section><h2>' + model_variant + '</h2>'
            '<img src="' + result['animation'] + '" alt="' + model_variant
            + ' MuJoCo settle and contact animation"></section>'
        )
    html = (
        '<!doctype html><meta charset="utf-8">'
        '<title>Robot Dog MuJoCo model review</title>'
        '<style>body{font:16px sans-serif;margin:2rem;background:#202428;'
        'color:#eee}main{display:flex;gap:2rem;flex-wrap:wrap}section{'
        'background:#30363b;padding:1rem;border-radius:.5rem}img{width:360px;'
        'height:270px}code{color:#9ee493}</style>'
        '<h1>Phase 3 deterministic model review</h1>'
        '<p>Feet turn green when the public contact interface reports '
        'contact. The enhanced model renders its passive closure paths in '
        'magenta. '
        'This visual artifact is supplemental; <code>manifest.json</code> '
        'records the deterministic public-data fingerprints.</p><main>'
        + ''.join(cards)
        + '</main>'
    )
    (output_directory / 'index.html').write_text(html, encoding='utf-8')


def generate_review_artifact(output_directory):
    """Generate review animations and deterministic metadata for all models."""
    output_directory = Path(output_directory)
    output_directory.mkdir(parents=True, exist_ok=True)
    manifest = {
        'camera': {
            'azimuth_degrees': 135.0,
            'distance': 0.75,
            'elevation_degrees': -20.0,
            'lookat': [0.0, 0.0, 0.12],
        },
        'contact_color_rgba': list(CONTACT_COLOR),
        'models': {},
        'note': (
            'Supplemental raster evidence; deterministic tests remain '
            'authoritative and rendering is not asserted bitwise portable.'
        ),
        'resolution': [FRAME_WIDTH, FRAME_HEIGHT],
    }
    for model_variant in MODEL_VARIANTS:
        manifest['models'][model_variant] = _record_variant(
            model_variant,
            output_directory,
        )
    _write_index(output_directory, manifest)
    (output_directory / 'manifest.json').write_text(
        json.dumps(manifest, indent=2, sort_keys=True) + '\n',
        encoding='utf-8',
    )
    return manifest


def main():
    """Generate both model animations in the requested directory."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('output_directory', type=Path)
    arguments = parser.parse_args()
    manifest = generate_review_artifact(arguments.output_directory)
    print(
        'MuJoCo review artifact generated for '
        + ', '.join(manifest['models'])
        + f' in {arguments.output_directory}'
    )


if __name__ == '__main__':
    main()
