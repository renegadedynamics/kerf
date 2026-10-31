#!/bin/sh
# Writes the Delabella instantiations of BRepMesh's node-insertion algorithms (BRepMesh_*MeshAlgoOfDelabella.cs) from
# the Watson generics, changing only the names (phase4-plan section 6, decision 548).
# Usage: sh tools/gen-brepmesh-delabella.sh src/OcctNet.ModelingAlgorithms/BRepMesh
set -e
cd "$1"
for n in NodeInsertionMeshAlgo DelaunayNodeInsertionMeshAlgo DelaunayDeflectionControlMeshAlgo; do
  perl -0pe '
    s{/// <remarks>.*?</remarks>\n}{/// <remarks>\n/// The Delabella instantiation of the OCCT template (over\n/// <c>BRepMesh_CustomDelaunayBaseMeshAlgo&lt;BRepMesh_DelabellaBaseMeshAlgo&gt;</c>), written by a script from the\n/// Watson generic\x27s body with only the names changed (phase4-plan section 6, decision 202\x27s approach).\n/// </remarks>\n}s;
    s/\bBRepMesh_(NodeInsertionMeshAlgo|DelaunayNodeInsertionMeshAlgo|DelaunayDeflectionControlMeshAlgo)</BRepMesh_$1OfDelabella</g;
    s/: BRepMesh_DelaunayBaseMeshAlgo\b/: BRepMesh_CustomDelaunayBaseMeshAlgoOfDelabella/g;
    s/(public|protected) BRepMesh_(NodeInsertionMeshAlgo|DelaunayNodeInsertionMeshAlgo|DelaunayDeflectionControlMeshAlgo)\(\)/$1 BRepMesh_$2OfDelabella()/g;
    s/Extends the Watson base algorithm/Extends the Delabella base algorithm/;
  ' "BRepMesh_$n.cs" > "BRepMesh_${n}OfDelabella.cs"
done
