out float FragColor;

void main()
{
    FragColor = float(gl_PrimitiveID + 1);
}