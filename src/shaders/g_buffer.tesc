#version 430 core
// define the number of CPs in the output patch
layout (vertices = 3) out;
// attributes of the input CPs
in V_OUT
{
    vec3 world_pos;
    vec3 world_normal;
    vec2 texture_pos;
    vec3 ambient;
    vec3 diffuse;
    vec3 specular;
    vec3 world_tagent;
} c_in[];

out C_OUT
{
    vec3 world_pos;
    vec3 world_normal;
    vec2 texture_pos;
    vec3 ambient;
    vec3 diffuse;
    vec3 specular;
    vec3 world_tagent;
} c_out[];


layout (std140, binding = 2) uniform View
{
    vec3 u_view_dir;    //0
    vec3 u_view_pos;    //16
};

float getTessLevel(float distance_0, float distance_1);

void main()
{
    // Set the control points of the output patch
    c_out[gl_InvocationID].world_pos = c_in[gl_InvocationID].world_pos;
    c_out[gl_InvocationID].world_normal = c_in[gl_InvocationID].world_normal;
    c_out[gl_InvocationID].texture_pos = c_in[gl_InvocationID].texture_pos;
    c_out[gl_InvocationID].ambient = c_in[gl_InvocationID].ambient;
    c_out[gl_InvocationID].diffuse = c_in[gl_InvocationID].diffuse;
    c_out[gl_InvocationID].specular = c_in[gl_InvocationID].specular;
    c_out[gl_InvocationID].world_tagent = c_in[gl_InvocationID].world_tagent;

    // Calculate the distance from the camera to the three control points
    float distance_0 = distance(u_view_pos, c_out[0].world_pos);
    float distance_1 = distance(u_view_pos, c_out[1].world_pos);
    float distance_2 = distance(u_view_pos, c_out[2].world_pos);

    // Calculate the tessellation levels
    gl_TessLevelOuter[0] = getTessLevel(distance_1, distance_2);
    gl_TessLevelOuter[1] = getTessLevel(distance_2, distance_0);
    gl_TessLevelOuter[2] = getTessLevel(distance_0, distance_1);
    gl_TessLevelInner[0] = gl_TessLevelOuter[2];

    // gl_TessLevelOuter[0] = 2.0;
    // gl_TessLevelOuter[1] = 2.0;
    // gl_TessLevelOuter[2] = 2.0;
    // gl_TessLevelInner[0] = 3.0;
}

float getTessLevel(float distance_0, float distance_1)
{
    float AvgDistance = (distance_0 + distance_1) / 2.0;
    if (AvgDistance <= 5.0) {
        return 7.0;
    }
    else if (AvgDistance <= 10.0) {
        return 3.0;
    }
    else {
        return 1.0;
    }
} 