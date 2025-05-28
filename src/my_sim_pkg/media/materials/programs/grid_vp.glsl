varying vec3 worldPos;
varying float depth;

void main()
{
  // Pass the world position to the fragment shader
  worldPos = gl_Vertex.xyz * 16.0; // Scale for a 16x16 grid
  gl_Position = gl_ModelViewProjectionMatrix * gl_Vertex;
  depth = gl_Position.z;
}