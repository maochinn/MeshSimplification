layout (location = 0) in vec3 position;

void main()
{
    gl_Position = u_projection * u_view * u_model * vec4(position, 1.0f);
}