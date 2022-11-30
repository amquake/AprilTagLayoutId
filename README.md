# AprilTagLayoutId

Estimates a layout of AprilTags by periodically recording the visible tags and their transformations between eachother.

To avoid redundant transformations causing conflicting layout reconstructions, the known transformations are represented in a graph structure and solved with Kruskal's algorithm for a Minimum Spanning Tree (MST).

`TagLayoutId.update(List)`:
> If multiple tags are visible together long enough, a transform is estimated between them-- the most consistent transforms previously seen are saved as unique edges between two tags. These edges are used to create a graph structure and solve for the Minimum Spanning Tree(Forest).

`TagLayoutId.findLayout(AprilTag)`:
> Uses the solved MSF from {@link #update(List)} to reconstruct an AprilTag layout without conflicts.
<br></br>An origin may be supplied to give a known point in the layout. If this origin is null or is not seen in the previously visible tags, an origin is instead picked as the tag with the lowest ID from the previously visible tags. This is important as the solved MSF may not be a MST, which means all tags may not be part of one connected tree. The tag layout is reconstructed starting from the origin, so only tags with connections to the origin tag will be returned in the layout.

The found layout can be serialized into a JSON for use in other projects.

-----

This project shows an example of this approach with a PhotonVision camera.
Here is a short video showcasing this:

https://user-images.githubusercontent.com/7953350/204755306-2fde6cc0-b734-4579-b41b-85dcb419d7d7.mp4

-----

To use this functionality in your own project, you'll need `Graph.java`, `TagLayoutId.java`, and `MathUtils.java` (if using a version older than wpilib-2023-beta-5, you'll also need `AprilTag.java` and `AprilTagFieldLayout.java`).
