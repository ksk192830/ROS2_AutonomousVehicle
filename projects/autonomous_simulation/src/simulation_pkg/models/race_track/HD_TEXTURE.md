# Exhibition display texture

`materials/textures/track_hd.png` is a 1448 × 1086 restoration of the existing
1181 × 885 `track.png`, made with the image generation tool on 2026-09-20.
The original remains untouched as the geometric reference for track-boundary
extraction. The new image is a generated restoration, not a higher-resolution
original from the author and not a 4K image.

Original public project: https://github.com/SKKUAutoLab/ros2_autonomous_vehicle_simulation
The upstream recursive tree contained the same `track.png` Git blob
`e857201825f64393ac16bbb3364733f05df6c483`; no higher-resolution/vector source
was found there. Model metadata credits Jinsun-Lee.

Two restoration candidates were compared at the original coordinate scale.
The selected candidate's asphalt ring mask overlapped the source by 98.37%
(IoU). Symmetric boundary distance was median 0 px and 95th percentile 2.20 px;
maximum 7.99 px, including line interruptions. This is approximate restoration,
not pixel-identical geometry. Comparison data is in
`reports/hd-track-20260920/geometry-comparison.json`.

`materials/exhibition/track_hd.material` uses a unique material and texture name.
The exhibition launch binds absolute local resource URIs, avoiding stale assets
under `~/.gazebo/models/race_track`. The legacy material's misspelled filtering
setting was also corrected. Anisotropic filtering preserves detail at grazing
view angles: https://ogrecave.github.io/ogre/api/13/_material-_scripts.html

Backups: `backups/before-hd-track-20260920/`.
