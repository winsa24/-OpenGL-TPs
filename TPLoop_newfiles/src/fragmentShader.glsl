#version 450 core            // minimal GL version support expected from the GPU

struct LightSource {
  vec3 position;
  vec3 color;
  float intensity;
  int isActive;
};

int numberOfLights = 3;
uniform LightSource lightSources[3];
uniform sampler2D shadowMapTex[3];
uniform mat4 shadowMapMVP[3];

struct Material {
  vec3 albedo;
  sampler2D albedoTex;
  int albedoTexLoaded;

  sampler2D normalTex;
  int normalTexLoaded;
};

uniform Material material;

uniform vec3 camPos;

in vec3 fPositionModel;
in vec3 fPosition;
in vec3 fNormal;
in vec2 fTexCoord;

out vec4 colorOut; // shader output: the color response attached to this fragment

uniform mat4 modelMat;

float pi = 3.1415927;
float shadowOffset = 0.001;

void main() {
  vec3 n = (material.normalTexLoaded == 1) ?
    normalize(mat3(modelMat)*((texture(material.normalTex, fTexCoord).rgb - 0.5)*2.0)) : // colors are in [0,1]^3, and normals are in [-1,1]^3
    normalize(fNormal);

  // linear barycentric interpolation does not preserve unit vectors
  vec3 wo = normalize(camPos - fPosition); // unit vector pointing to the camera

  vec3 radiance = vec3(0, 0, 0);
  for(int i=0; i<numberOfLights; ++i) {
    LightSource a_light = lightSources[i];
    if(a_light.isActive == 1) { // consider active lights only
      vec4 ShadowCoord = shadowMapMVP[i]*modelMat*vec4(fPositionModel, 1);
      ShadowCoord /= ShadowCoord.w;

      // so far, ShadowCoord is in [-1,1]^3, put it in [0,1]^3:
      ShadowCoord = ShadowCoord*0.5 + 0.5;

      if(texture(shadowMapTex[i], ShadowCoord.xy).r < ShadowCoord.z - shadowOffset) {
        // useful : debug
        // radiance = vec3(ShadowCoord2D, 0);
      } else {
        vec3 wi = normalize(a_light.position - fPosition); // unit vector pointing to the light source
        vec3 Li = a_light.color*a_light.intensity;
        vec3 albedo = material.albedoTexLoaded==1 ? texture(material.albedoTex, fTexCoord).rgb : material.albedo;

        radiance += Li*albedo*max(dot(n, wi), 0);
      }
    }
  }

  {
    // !!!!!! DEBUG YOUR TEXTURES !!!!!! :
    //   radiance = vec3(1) * max(dot(n,wo),0.0);
    //   radiance = 0.001*radiance + 0.999*((texture(material.normalTex, fTexCoord).rgb));
    //   radiance = 0.001*radiance + 0.999*((texture(material.albedoTex, fTexCoord).rgb));
  }

  colorOut = vec4(radiance, 1.0); // build an RGBA value from an RGB one
}
