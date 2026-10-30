# Runtime AI models (not distributed)

The source code expects `track.pt`, `parking_front.pt`, `parking_rear.pt`, `crosswalk.pt`, and `traffic_light_sim.pt`, but **the trained weights are not included in this public repository**.

Use your own trained weights or retrieve authorized originals locally. Model files are Git-ignored and should not be committed or redistributed.

Real-vehicle configurations also require TensorRT engines (`track.engine`, `parking_front.engine`), which must be built for the target environment. YOLO `.pt` weights are not interchangeable with an RF-DETR TensorRT engine.

See the [reproduction guide](../../../../../../docs/reproduction.md).
