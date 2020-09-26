#version 430 core
layout(triangles, equal_spacing, ccw) in;

in C_OUT
{
    vec3 world_pos;
    vec3 world_normal;
    vec2 texture_pos;
    vec3 ambient;
    vec3 diffuse;
    vec3 specular;
    vec3 world_tagent;
} e_in[];

out E_OUT
{
    vec3 world_pos;
    vec3 world_normal;
    vec2 texture_pos;
    vec3 ambient;
    vec3 diffuse;
    vec3 specular;
    mat3 TBN;
} e_out;

layout (std140, binding = 0) uniform Matrices
{
    mat4 u_projection;
    mat4 u_view;
};

struct Material
{
    sampler2D texture_diffuse;
    sampler2D texture_specular;
    sampler2D texture_normal;
    sampler2D texture_displacement;
    samplerCube texture_cubemap;
    float shininess;
};
uniform Material u_material;

uniform bool u_use_height_map;
uniform bool u_use_displacement_map;
uniform mat4 u_model;
uniform vec2 u_texture_size;

vec2 interpolate2D(vec2 v0, vec2 v1, vec2 v2);
vec3 interpolate3D(vec3 v0, vec3 v1, vec3 v2);

void getHeightAndNormal(out float height, out vec3 normal, in vec2 texture_pos)
{
    vec4 texel = texture(u_material.texture_displacement, texture_pos);
    
    //calculate normal
    ivec3 off = ivec3(-1,0,1);
    vec2 size = 1.0f / u_texture_size;

    float s11 = texel.x;
    float s01 = textureOffset(u_material.texture_displacement, texture_pos, off.xy).r;
    float s21 = textureOffset(u_material.texture_displacement, texture_pos, off.zy).r;
    float s10 = textureOffset(u_material.texture_displacement, texture_pos, off.yx).r;
    float s12 = textureOffset(u_material.texture_displacement, texture_pos, off.yz).r;
    vec3 va = normalize(vec3(size.xy,s21-s01));
    vec3 vb = normalize(vec3(size.yx,s12-s10));
    //vec3 va = normalize(vec3(size.x ,s21-s01, size.y));
    //vec3 vb = normalize(vec3(size.x, s12-s10, size.y));
   
    height = texel.x - 0.5;   //normalize to [-0.5, 0.5]
    normal = cross(va,vb);
}

void main()
{
    // Interpolate the attributes of the output vertex using the barycentric coordinates
    e_out.world_pos = interpolate3D(e_in[0].world_pos, e_in[1].world_pos, e_in[2].world_pos);
    e_out.texture_pos = interpolate2D(e_in[0].texture_pos, e_in[1].texture_pos, e_in[2].texture_pos);
    e_out.ambient = interpolate3D(e_in[0].ambient, e_in[1].ambient, e_in[2].ambient);
    e_out.diffuse = interpolate3D(e_in[0].diffuse, e_in[1].diffuse, e_in[2].diffuse);
    e_out.specular = interpolate3D(e_in[0].specular, e_in[1].specular, e_in[2].specular);

    vec3 T = normalize(interpolate3D(e_in[0].world_tagent, e_in[1].world_tagent, e_in[2].world_tagent));
    vec3 N = normalize(interpolate3D(e_in[0].world_normal, e_in[1].world_normal, e_in[2].world_normal));
    T = normalize(T - dot(T, N) * N);
    vec3 B = cross(N, T);   
    e_out.TBN = mat3(T, B, N);

    e_out.world_normal = N;

    if (u_use_height_map)
    {
        float height;
        vec3 normal;

        getHeightAndNormal(height, normal, e_out.texture_pos);
        e_out.world_pos += e_out.world_normal * height * 5.0;
        e_out.world_normal = transpose(inverse(mat3(u_model))) * normal;
    }

    gl_Position = u_projection * u_view * vec4(e_out.world_pos, 1.0);
}

vec2 interpolate2D(vec2 v0, vec2 v1, vec2 v2)
{
    return vec2(gl_TessCoord.x) * v0 + vec2(gl_TessCoord.y) * v1 + vec2(gl_TessCoord.z) * v2;
}
vec3 interpolate3D(vec3 v0, vec3 v1, vec3 v2)
{
    return vec3(gl_TessCoord.x) * v0 + vec3(gl_TessCoord.y) * v1 + vec3(gl_TessCoord.z) * v2;
} 