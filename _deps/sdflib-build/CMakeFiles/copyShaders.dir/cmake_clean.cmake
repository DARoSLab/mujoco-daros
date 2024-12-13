file(REMOVE_RECURSE
  "basic.frag"
  "basic.vert"
  "colors.frag"
  "colors.vert"
  "grid.frag"
  "grid.vert"
  "normals.frag"
  "normals.vert"
  "normalsSplitPlane.frag"
  "normalsSplitPlane.vert"
  "screenPlane.frag"
  "screenPlane.vert"
  "sdfOctreeLight.frag"
  "sdfOctreeLight.vert"
  "sdfOctreeMeanTrianglesPlane.frag"
  "sdfOctreeMeanTrianglesPlane.vert"
  "sdfOctreePlane.frag"
  "sdfOctreePlane.vert"
  "sdfOctreeRender.comp"
  "sdfPlane.frag"
  "sdfPlane.vert"
)

# Per-language clean rules from dependency scanning.
foreach(lang )
  include(CMakeFiles/copyShaders.dir/cmake_clean_${lang}.cmake OPTIONAL)
endforeach()
