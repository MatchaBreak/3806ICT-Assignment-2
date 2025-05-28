varying vec3 worldPos;
varying float depth;

void main()
{
  // Size of each grid cell
  float gsize = 1.0;        // size is 1x1 units (xy)

  // Width of the grid lines
  float gwidth = 0.05;

  // Calculate the grid pattern
  vec2 grid = abs(fract(worldPos.xy / gsize)) / gwidth;
  float line = step(min(grid.x, grid.y), 0.5);

  // Base grey color
  vec3 baseColor = vec3(0.68, 0.85, 0.9); // Light blue background

  // Grid line color (black)
  vec3 gridColor = vec3(0.0, 0.0, 0.0);

  // Mix the base color and grid lines
  vec3 finalColor = mix(baseColor, gridColor, line);

  gl_FragColor = vec4(finalColor, 1.0);
}

