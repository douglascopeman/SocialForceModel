#!/usr/bin/env python3
"""Convert sfm_sim's timestamped SVG frames to MP4 and GIF using local tools."""

import argparse
from pathlib import Path
import re
import shutil
import subprocess


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("run_directory", type=Path)
    parser.add_argument("--fps", type=int, default=20, help="Playback frames/second (default: 20)")
    args = parser.parse_args()
    if args.fps <= 0:
        parser.error("--fps must be positive")
    for tool in ("rsvg-convert", "ffmpeg"):
        if not shutil.which(tool):
            parser.error(f"{tool} is required; install it to create animations")

    run_dir = args.run_directory.resolve()
    frames = []
    for path in run_dir.glob("frame_*ms.svg"):
        match = re.fullmatch(r"frame_(\d+)ms\.svg", path.name)
        if match:
            frames.append((int(match[1]), path))
    frames.sort()
    if len(frames) < 2:
        parser.error("at least two timestamped SVG frames are required")
    intervals = [later[0] - earlier[0] for earlier, later in zip(frames, frames[1:])]
    if max(intervals) - min(intervals) > 1:
        parser.error("frames must be uniformly spaced in simulation time")

    png_dir = run_dir / "animation_frames"
    png_dir.mkdir(exist_ok=True)
    for index, (_, svg) in enumerate(frames):
        png = png_dir / f"frame_{index:05d}.png"
        if not png.exists() or png.stat().st_mtime < svg.stat().st_mtime:
            subprocess.run(["rsvg-convert", str(svg), "-o", str(png)], check=True)

    movie = run_dir / "evolution.mp4"
    subprocess.run([
        "ffmpeg", "-hide_banner", "-loglevel", "error", "-y",
        "-framerate", str(args.fps), "-i", str(png_dir / "frame_%05d.png"),
        "-frames:v", str(len(frames)), "-vf", "pad=ceil(iw/2)*2:ceil(ih/2)*2",
        "-c:v", "libx264", "-preset", "fast", "-crf", "23", "-threads", "2",
        "-pix_fmt", "yuv420p", "-movflags", "+faststart", str(movie),
    ], check=True)
    gif = run_dir / "evolution.gif"
    subprocess.run([
        "ffmpeg", "-hide_banner", "-loglevel", "error", "-y", "-i", str(movie),
        "-filter_complex_threads", "1", "-filter_complex",
        "fps=10,scale=720:-1:flags=lanczos,split[a][b];"
        "[a]palettegen[p];[b][p]paletteuse=dither=bayer", "-loop", "0", str(gif),
    ], check=True)
    acceleration = intervals[0] / 1000.0 * args.fps
    print(f"Created {movie} and {gif}")
    print(f"{len(frames)} frames; {args.fps} fps; approximately {acceleration:g}x real time")


if __name__ == "__main__":
    main()
