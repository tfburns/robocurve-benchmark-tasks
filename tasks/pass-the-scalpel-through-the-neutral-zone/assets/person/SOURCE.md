# Person asset: source and licence

- Model: Microsoft Rocketbox Avatar Library, avatar `Medical_Male_04` (`Assets/Avatars/Professions/Medical_Male_04/Export/Medical_Male_04.fbx`
  and its `_color` textures: green surgical scrubs and a surgical mask), https://github.com/microsoft/Microsoft-Rocketbox
  (`master`, downloaded 16 Nov 2026).
- Licence: MIT (`LICENSE-Rocketbox.md`, Copyright (c) 2020 Microsoft). The MIT licence allows modifying and
  redistributing the model, so the posed copy is shipped here.
- Changes: posed once in Blender 4.2 (standing just past the left-hand end of the table, leaning in 10 degrees, both
  hands held together in front of him about 1.0-1.1 m up, see `colleague_proxies.json` "pose"), the skinning baked
  into a static mesh, the skeleton removed, colour textures downscaled to 1024 / 512 px JPEG, normal and specular maps
  dropped, invisible collision capsules (arms, torso, head, legs) added, the whole marked as one kinematic (fixed)
  rigid body. His hands are bare skin: the avatar has no surgical gloves or gown.
- Why not NVIDIA's Isaac Sim people (`Isaac/People/Characters`): they are covered by the NVIDIA Isaac Sim Additional
  Software and Materials Licence, which does not allow modifying or redistributing them, and this task has to ship a
  posed static copy.
