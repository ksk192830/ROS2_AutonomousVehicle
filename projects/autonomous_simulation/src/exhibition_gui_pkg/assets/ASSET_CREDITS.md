# Steering-wheel asset

- File: `steering_wheel.png`
- Source: Wikimedia Commons, `Steering-wheel.svg`
- Author: gnokii / Open Clip Art Library
- License: CC0 1.0 (public domain dedication)
- Source page: https://commons.wikimedia.org/wiki/File:Steering-wheel.svg

The Wikimedia 960 px PNG rendering is bundled without visual modification.

# Perception lesson examples

`examples/source-camera.png`, `source-segmentation.png`, and `source-bev.png`
were captured from this project's exhibition simulator on 2026-09-19. All
three share source stamp 10.357 seconds. They retain the project's existing
simulation assets and model outputs; no external video or stock image was added.
`recording.json` records the perspective parameters used by the lane detector.
The GUI animates the recorded images as a clearly labeled teaching example,
independent of the live driving and perception streams.

# Additional beginner examples (2026-09-19)

## Object regions on a desk

- Files: `examples/desk-original.jpg`, `examples/desk-segmented.png` (unmodified originals).
- Author: Martin Thoma; source: own photographs and region annotations, 2016.
- License: CC0 1.0, https://creativecommons.org/publicdomain/zero/1.0/
- Original: https://commons.wikimedia.org/wiki/File:Image-segmentation-example.jpg
- Region annotation: https://commons.wikimedia.org/wiki/File:Image-segmentation-example-segmented.png
- The GUI reveals the provided annotation next to the original. This is labeled
  as a human-provided learning example, not output from the exhibition's model.

## Rectifying a photographed paper grid

- File: `examples/sudoku.png` (unmodified OpenCV sample).
- Source: https://github.com/opencv/opencv/blob/4.x/samples/data/sudoku.png
- Tutorial: https://docs.opencv.org/4.x/da/d6e/tutorial_py_geometric_transformations.html
- License: OpenCV Apache 2.0; included as `examples/OPENCV_LICENSE.txt`.
- The GUI applies an animated perspective transform to the four grid corners.
  This illustrates the plane-view transformation also used in this exhibit's
  BEV pipeline. It is not a new road camera, 3D reconstruction, or a driving input.

Additional research consulted: Cityscapes examples
(https://www.cityscapes-dataset.com/examples/) and Nissan's Around View Monitor
(https://www.nissan-global.com/EN/INNOVATION/TECHNOLOGY/ARCHIVE/IAVM/).
Their images are not bundled. The existing exhibit uses one forward image for
its road-plane BEV; it does not implement a four-camera surround-view system.
