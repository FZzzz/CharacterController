#version 330 core
out vec4 FragColor;

in VS_OUT {
    vec3 FragPos;
    vec2 TexCoords;
    vec4 FragPosLightSpace;
    vec3 Color;
} fs_in;

//uniform sampler2D diffuseTexture;
//uniform sampler2D shadowMap;

// directional light
uniform vec3 lightPos;
uniform vec3 viewPos;

void main()
{           
    //vec3 color = texture(diffuseTexture, fs_in.TexCoords).rgb;
    vec3 color = fs_in.Color;
    vec3 fdx = dFdx(fs_in.FragPos);
    vec3 fdy = dFdy(fs_in.FragPos);
    vec3 normal = normalize(cross(fdx, fdy));
    vec3 lightColor = vec3(1);
    // ambient
    vec3 ambient = 0.1 * color;
    // diffuse
    vec3 lightDir = normalize(lightPos - fs_in.FragPos);
    //vec3 lightDir = normalize(vec3(5.0f , 5.0f , 5.0f));
    float diff = max(dot(lightDir, normal), 0.0);
    vec3 diffuse = diff * lightColor;
    // specular
    vec3 viewDir = normalize(viewPos - fs_in.FragPos);
    vec3 reflectDir = reflect(-lightDir, normal);
    float spec = 0.0;
    vec3 halfwayDir = normalize(lightDir + viewDir);  
    spec = pow(max(dot(normal, halfwayDir), 0.0), 64.0);
    vec3 specular = spec * lightColor;    
    // calculate shadow
    //float shadow = ShadowCalculation(fs_in.FragPosLightSpace, lightDir, normal);                      
    vec3 lighting = (ambient + diffuse + specular) * color;
   
    //vec3 lighting = (ambient + diffuse) * color;

    FragColor = vec4(lighting, 1.0);
}




/*
void main()
{

    //vec3 light_pos = vec3(5.0f , 5.0f , 5.0f);
	vec3 light_color = vec3(1.0f);
	vec3 object_color = vec3(1.0f , 1.0f , 1.0f);

    // ambient
    float ambientStrength = 0.1;
    vec3 ambient = ambientStrength * light_color;
  	
    // diffuse
	vec3 fdx = dFdx(fs_in.FragPos);
	vec3 fdy = dFdy(fs_in.FragPos);
	vec3 normal = normalize(cross(fdx , fdy));
    vec3 litDir = normalize(vec3(5.0f , 5.0f , 5.0f));
    float diff = max(dot(normal, litDir), 0.0);
    vec3 diffuse = diff * light_color;
    
	// (x, y, z) * (a, b, c) = (ax, by, cz)
    vec3 result = (ambient + diffuse) * object_color;
    fragColor = vec4(result, 1.0);
} 
*/