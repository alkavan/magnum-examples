uniform mat3 matrix;

layout(location = 0) in vec2 vertex;

out vec2 textureCoords;

void main() {
    textureCoords = vertex/2+vec2(0.5);
    gl_Position.xywz = vec4(matrix*vec3(vertex, 1.0), 0.0);
}
