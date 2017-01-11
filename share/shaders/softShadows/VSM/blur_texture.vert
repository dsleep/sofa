#version 120
varying vec3 N;
varying vec3 lightDir;
varying vec4 lightVec;
uniform vec4 lightPosition;
varying vec3 viewVector;
varying vec3 lightpos;
uniform vec3 campos;
varying vec4 pos;
void main()
{	
  pos = gl_ModelViewMatrix * gl_Vertex;
  vec3 lightpos = vec3(-6.281678199768066, -3.287038803100586, 0.9171836376190186);//gl_LightSource[0].position.xyz;
  //position="-6.281678199768066 -3.287038803100586 0.9171836376190186 "
	//lp = lightPosition;  
	//vec4 ecPos = gl_ModelViewMatrix * gl_Vertex;
	//lightVec = ecPos-lp;
  //lightVec = lp;
  lightDir =  lightpos - pos.xyz;
  viewVector = pos.xyz;//gl_ModelViewMatrix[3].xyz );//-( campos - gl_Vertex.xyz );
  
	gl_TexCoord[0] = gl_MultiTexCoord0;
	gl_Position = ftransform();
  vec4 Nr = gl_ModelViewMatrixInverseTranspose * vec4(gl_Normal.xyz, 1);
  N = normalize(Nr.xyz);
}
