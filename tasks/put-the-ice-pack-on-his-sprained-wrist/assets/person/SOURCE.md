# Person asset: source and licence

- Model: Microsoft Rocketbox Avatar Library, avatar `Sports_Male_02` (`Assets/Avatars/Professions/Sports_Male_02/Export/Sports_Male_02.fbx`
  and its `_color` textures), https://github.com/microsoft/Microsoft-Rocketbox (commit on `master`, downloaded
  16 Nov 2026). A footballer in a short-sleeved striped kit shirt, so the forearms are bare.
- Licence: MIT (`LICENSE-Rocketbox.md`, Copyright (c) 2020 Microsoft). The MIT licence allows modifying and
  redistributing the model, so the posed copy is shipped here.
- Changes: posed once in Blender 4.2 (seated at the end of the table, leaning forward, right forearm lying diagonally
  on the table, left hand on the table; see `colleague_proxies.json` "pose"), the skinning baked into a static mesh,
  the skeleton removed, colour textures downscaled to JPEG, normal and specular maps dropped, invisible collision
  shapes (capsules for arms, torso, head and legs; a box for each hand) added, the whole marked as one kinematic
  (fixed) rigid body.
- Collider fit: the automatic capsule radius (90th percentile of skin distance) made the right forearm and the elbow
  end of the right upper arm too fat for a task that touches them, so these two capsules were refitted by hand to the
  mesh's skin top (right forearm: radius 0.035 m, axis from (0.138, 0.561, 0.811) to (0.33, 0.37, 0.805); right upper arm: radius 0.045 m at the elbow). The same values are in `colleague_proxies.json`.
- Why not NVIDIA's Isaac Sim people (`Isaac/People/Characters`): they are covered by the NVIDIA Isaac Sim Additional
  Software and Materials Licence, which does not allow modifying or redistributing them, and this task has to ship a
  posed static copy.
