/* **************************
 * CSCI 420
 * Assignment 3 Raytracer
 * Name: Anna Zhu
 * *************************
*/

#ifdef WIN32
  #include <windows.h>
#endif

#if defined(WIN32) || defined(linux)
  #include <GL/gl.h>
  #include <GL/glut.h>
#elif defined(__APPLE__)
  #include <OpenGL/gl.h>
  #include <GLUT/glut.h>
#endif

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <time.h> // for rand
#ifdef WIN32
  #define strcasecmp _stricmp
#endif

#include <imageIO.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define MAX_TRIANGLES 20000
#define MAX_SPHERES 100
#define MAX_LIGHTS 100
#define MAX_DEPTH 3 // max bounces for reflections
#define SHADOW_SAMPLES 8 // samples per light for soft shadows
#define LIGHT_RADIUS 0.2 // area light radius

char * filename = NULL;

// The different display modes.
#define MODE_DISPLAY 1
#define MODE_JPEG 2

int mode = MODE_DISPLAY;

// While solving the homework, it is useful to make the below values smaller for debugging purposes.
// The still images that you need to submit with the homework should be at the below resolution (640x480).
// However, for your own purposes, after you have solved the homework, you can increase those values to obtain higher-resolution images.
#define WIDTH 640
#define HEIGHT 480

// The field of view of the camera, in degrees.
#define fov 60.0

// Buffer to store the image when saving it to a JPEG.
unsigned char buffer[HEIGHT][WIDTH][3];

struct Vertex
{
  double position[3];
  double color_diffuse[3];
  double color_specular[3];
  double normal[3];
  double shininess;
};

struct Triangle
{
  Vertex v[3];
};

struct Sphere
{
  double position[3];
  double color_diffuse[3];
  double color_specular[3];
  double shininess;
  double radius;
};

struct Light
{
  double position[3];
  double color[3];
};

Triangle triangles[MAX_TRIANGLES];
Sphere spheres[MAX_SPHERES];
Light lights[MAX_LIGHTS];
double ambient_light[3];

int num_triangles = 0;
int num_spheres = 0;
int num_lights = 0;

void plot_pixel_display(int x,int y,unsigned char r,unsigned char g,unsigned char b);
void plot_pixel_jpeg(int x,int y,unsigned char r,unsigned char g,unsigned char b);
void plot_pixel(int x,int y,unsigned char r,unsigned char g,unsigned char b);

//vec helpers
void vec_add(const double a[3], const double b[3], double r[3])
{
    r[0] = a[0] + b[0];
    r[1] = a[1] + b[1];
    r[2] = a[2] + b[2];
}

void vec_sub(const double a[3], const double b[3], double r[3])
{
    r[0] = a[0] - b[0];
    r[1] = a[1] - b[1];
    r[2] = a[2] - b[2];
}

void vec_scale(const double a[3], double s, double r[3])
{
    r[0] = a[0] * s;
    r[1] = a[1] * s;
    r[2] = a[2] * s;
}

void vec_copy(const double a[3], double r[3])
{
    r[0] = a[0]; r[1] = a[1]; r[2] = a[2];
}

void vec_set(double v[3], double x, double y, double z)
{
    v[0] = x; v[1] = y; v[2] = z;
}

double vec_dot(const double a[3], const double b[3])
{
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

void vec_cross(const double a[3], const double b[3], double r[3])
{
    r[0] = a[1] * b[2] - a[2] * b[1];
    r[1] = a[2] * b[0] - a[0] * b[2];
    r[2] = a[0] * b[1] - a[1] * b[0];
}

double vec_length(const double a[3])
{
    return sqrt(vec_dot(a, a));
}

static const double EPSILON = 1e-6;

void vec_normalize(double a[3])
{
    double len = vec_length(a);
    if (len > EPSILON)
    {
        a[0] /= len;
        a[1] /= len;
        a[2] /= len;
    }
}

void vec_reflect(const double L[3], const double N[3], double R[3])
{
    double ndotl = vec_dot(N, L);
    double tmp[3];
    vec_scale(N, 2.0 * ndotl, tmp);
    vec_sub(tmp, L, R);
}

struct HitInfo
{
    bool hit;
    double t;
    double point[3];
    double normal[3];

    double kd[3];
    double ks[3];
    double shininess;
};

double rand01()
{
    return rand() / (double)RAND_MAX;
}

void sample(double radius, double offset[3])
{
    double r = radius * sqrt(rand01());       // sqrt for uniform area distribution
    double theta = 2.0 * M_PI * rand01();
    offset[0] = r * cos(theta);
    offset[1] = r * sin(theta);
    offset[2] = 0.0;
}

// sphere intersection
bool intersect_sphere(const double rayOrigin[3], const double rayDir[3],
    const Sphere& s, double& tHit)
{
    double oc[3];
    vec_sub(rayOrigin, s.position, oc);

    double a = vec_dot(rayDir, rayDir);
    double b = 2.0 * vec_dot(rayDir, oc);
    double c = vec_dot(oc, oc) - s.radius * s.radius;

    double disc = b * b - 4.0 * a * c;
    if (disc < 0.0) return false;

    double sqrtDisc = sqrt(disc);
    double t0 = (-b - sqrtDisc) / (2.0 * a);
    double t1 = (-b + sqrtDisc) / (2.0 * a);

    double t = t0;
    if (t < EPSILON) t = t1;
    if (t < EPSILON) return false;

    tHit = t;
    return true;
}

// triangle intersection
bool intersect_triangle(const double rayOrigin[3], const double rayDir[3],
    const Triangle& tri, double& tHit, double& u, double& v)
{
    const double* p0 = tri.v[0].position;
    const double* p1 = tri.v[1].position;
    const double* p2 = tri.v[2].position;

    double edge1[3], edge2[3];
    vec_sub(p1, p0, edge1);
    vec_sub(p2, p0, edge2);

    double pvec[3];
    vec_cross(rayDir, edge2, pvec);

    double det = vec_dot(edge1, pvec);
    if (fabs(det) < EPSILON) return false;

    double invDet = 1.0 / det;

    double tvec[3];
    vec_sub(rayOrigin, p0, tvec);

    u = vec_dot(tvec, pvec) * invDet;
    if (u < 0.0 || u > 1.0) return false;

    double qvec[3];
    vec_cross(tvec, edge1, qvec);

    v = vec_dot(rayDir, qvec) * invDet;
    if (v < 0.0 || u + v > 1.0) return false;

    double t = vec_dot(edge2, qvec) * invDet;
    if (t < EPSILON) return false;

    tHit = t;
    return true;
}

// find closest intersection
bool find_nearest_hit(const double rayOrigin[3], const double rayDir[3], HitInfo& hit)
{
    hit.hit = false;
    hit.t = 1e30;

    // sphere
    for (int i = 0; i < num_spheres; ++i)
    {
        double tSphere;
        if (intersect_sphere(rayOrigin, rayDir, spheres[i], tSphere))
        {
            if (tSphere < hit.t)
            {
                hit.hit = true;
                hit.t = tSphere;

                // point
                double scaledDir[3];
                vec_scale(rayDir, tSphere, scaledDir);
                vec_add(rayOrigin, scaledDir, hit.point);

                // normal
                vec_sub(hit.point, spheres[i].position, hit.normal);
                vec_normalize(hit.normal);

                // phong
                for (int c = 0; c < 3; ++c)
                {
                    hit.kd[c] = spheres[i].color_diffuse[c];
                    hit.ks[c] = spheres[i].color_specular[c];
                }
                hit.shininess = spheres[i].shininess;
            }
        }
    }

    // triangles
    for (int i = 0; i < num_triangles; ++i)
    {
        double tTri, u, v;
        if (intersect_triangle(rayOrigin, rayDir, triangles[i], tTri, u, v))
        {
            if (tTri < hit.t)
            {
                hit.hit = true;
                hit.t = tTri;

                double w = 1.0 - u - v;

                // positions
                double p0[3], p1[3], p2[3];
                vec_copy(triangles[i].v[0].position, p0);
                vec_copy(triangles[i].v[1].position, p1);
                vec_copy(triangles[i].v[2].position, p2);

                for (int c = 0; c < 3; ++c)
                {
                    hit.point[c] = w * p0[c] + u * p1[c] + v * p2[c];
                }

                // normals
                double n0[3], n1[3], n2[3];
                vec_copy(triangles[i].v[0].normal, n0);
                vec_copy(triangles[i].v[1].normal, n1);
                vec_copy(triangles[i].v[2].normal, n2);
                for (int c = 0; c < 3; ++c)
                {
                    hit.normal[c] = w * n0[c] + u * n1[c] + v * n2[c];
                }
                vec_normalize(hit.normal);

                // phong
                for (int c = 0; c < 3; ++c)
                {
                    hit.kd[c] =
                        w * triangles[i].v[0].color_diffuse[c] +
                        u * triangles[i].v[1].color_diffuse[c] +
                        v * triangles[i].v[2].color_diffuse[c];

                    hit.ks[c] =
                        w * triangles[i].v[0].color_specular[c] +
                        u * triangles[i].v[1].color_specular[c] +
                        v * triangles[i].v[2].color_specular[c];
                }

                hit.shininess =
                    w * triangles[i].v[0].shininess +
                    u * triangles[i].v[1].shininess +
                    v * triangles[i].v[2].shininess;
            }
        }
    }

    /*
    if (hit.hit)
    {
        double viewDir[3];
        // camera at origin, so viewDir = -point
        vec_scale(hit.point, -1.0, viewDir);
        if (vec_dot(hit.normal, viewDir) < 0.0)
        {
            hit.normal[0] = -hit.normal[0];
            hit.normal[1] = -hit.normal[1];
            hit.normal[2] = -hit.normal[2];
        }
    }
    */

    return hit.hit;
}

// lower level occlusion check for soft shadows
bool occluded_to_point(const double point[3], const double normal[3], const double targetPos[3])
{
    // offset slightly
    double origin[3];
    double offset[3];
    vec_scale(normal, EPSILON * 10.0, offset);
    vec_add(point, offset, origin);

    double dir[3];
    vec_sub(targetPos, origin, dir);
    double distToTarget = vec_length(dir);
    vec_normalize(dir);

    // spheres
    for (int i = 0; i < num_spheres; ++i)
    {
        double t;
        if (intersect_sphere(origin, dir, spheres[i], t))
        {
            if (t > EPSILON && t < distToTarget - EPSILON)
            {
                return true;
            }
        }
    }

    // triangles
    for (int i = 0; i < num_triangles; ++i)
    {
        double t, u, v;
        if (intersect_triangle(origin, dir, triangles[i], t, u, v))
        {
            if (t > EPSILON && t < distToTarget - EPSILON)
            {
                return true;
            }
        }
    }

    return false;
}

// shadow test
bool is_in_shadow(const double point[3], const double normal[3], const Light& light)
{
    return occluded_to_point(point, normal, light.position);
}

// 1 = fully lit, 0 = fully in shadow
double soft_shadow_visibility(const double point[3], const double normal[3], const Light& light)
{
    int visibleCount = 0;

    for (int s = 0; s < SHADOW_SAMPLES; ++s)
    {
        double offset[3];
        sample(LIGHT_RADIUS, offset);

        double samplePos[3];
        samplePos[0] = light.position[0] + offset[0];
        samplePos[1] = light.position[1] + offset[1];
        samplePos[2] = light.position[2] + offset[2]; 

        if (!occluded_to_point(point, normal, samplePos))
            visibleCount++;
    }

    return (double)visibleCount / (double)SHADOW_SAMPLES;
}

// local phong shading w/o reflection
void shade_local(const HitInfo& hit, const double viewDir[3], int depth, double outColor[3])
{
    // ambient
    for (int c = 0; c < 3; ++c)
    {
        outColor[c] = ambient_light[c] * hit.kd[c];
    }

    // add diffuse + specular (if not in shadow) for each light
    for (int i = 0; i < num_lights; ++i)
    {
        double visibility = 1.0;

        if (depth == 0)
        {
            visibility = soft_shadow_visibility(hit.point, hit.normal, lights[i]);
            if (visibility <= 0.0)
                continue; 
        }

        else
        {
            visibility = 1.0;
        }

        // point to light
        double L[3];
        vec_sub(lights[i].position, hit.point, L);
        vec_normalize(L);

        double NdotL = vec_dot(hit.normal, L);
        if (NdotL < 0.0) NdotL = 0.0;

        // reflection of -L around N  
        double Lneg[3] = { -L[0], -L[1], -L[2] };
        double R[3];
        vec_reflect(Lneg, hit.normal, R);
        vec_normalize(R);

        double V[3];
        vec_copy(viewDir, V);
        vec_normalize(V);

        double RdotV = vec_dot(R, V);
        if (RdotV < 0.0) RdotV = 0.0;

        double specTerm = 0.0;
        if (RdotV > 0.0 && hit.shininess > 0.0)
            specTerm = pow(RdotV, hit.shininess);

        for (int c = 0; c < 3; ++c)
        {
            double diffuse = hit.kd[c] * NdotL;
            double specular = hit.ks[c] * specTerm;
            outColor[c] += visibility * lights[i].color[c] * (diffuse + specular); // scale w visibility
        }
    }

    // clamp
    for (int c = 0; c < 3; ++c)
    {
        if (outColor[c] < 0.0) outColor[c] = 0.0;
        if (outColor[c] > 1.0) outColor[c] = 1.0;
    }
}

// recursive raytracer
void trace_ray(const double origin[3], const double dir[3], int depth, unsigned char outColor[3])
{
    // base case: too deep → no contribution
    if (depth > MAX_DEPTH)
    {
        outColor[0] = outColor[1] = outColor[2] = 0;
        return;
    }

    HitInfo hit;
    if (!find_nearest_hit(origin, dir, hit))
    {
        // background color
        double bg[3] = { 0.02, 0.02, 0.02 }; // tweak if you like
        outColor[0] = (unsigned char)(bg[0] * 255.0);
        outColor[1] = (unsigned char)(bg[1] * 255.0);
        outColor[2] = (unsigned char)(bg[2] * 255.0);
        return;
    }

    // local phong
    double viewDir[3] = { -dir[0], -dir[1], -dir[2] }; 
    double localColor[3];
    shade_local(hit, viewDir, depth, localColor);

    // ~ recursive reflection ~
    double reflectivity = hit.ks[0];
    if (hit.ks[1] > reflectivity) reflectivity = hit.ks[1];
    if (hit.ks[2] > reflectivity) reflectivity = hit.ks[2];

    // clamp reflectivity so it doesn't go crazy
    if (reflectivity < 0.0) reflectivity = 0.0;
    if (reflectivity > 1.0) reflectivity = 1.0;

    double finalColor[3] = { localColor[0], localColor[1], localColor[2] };

    if (reflectivity > 1e-3 && depth < MAX_DEPTH)
    {
        // R = D - 2(D·N)N
        double NdotD = vec_dot(hit.normal, dir);
        double reflectionDir[3];
        double tmp[3];

        vec_scale(hit.normal, 2.0 * NdotD, tmp);
        vec_sub(dir, tmp, reflectionDir);
        vec_normalize(reflectionDir);

        // offset a bit
        double newOrigin[3];
        double offset[3];
        vec_scale(hit.normal, EPSILON * 10.0, offset);
        vec_add(hit.point, offset, newOrigin);

        unsigned char reflU8[3];
        trace_ray(newOrigin, reflectionDir, depth + 1, reflU8);

        double reflColor[3] = {
          reflU8[0] / 255.0,
          reflU8[1] / 255.0,
          reflU8[2] / 255.0
        };

        // mix local and reflection
        for (int c = 0; c < 3; ++c)
        {
            finalColor[c] = (1.0 - reflectivity) * finalColor[c] + reflectivity * reflColor[c];
        }
    }

    // clamp/convert
    for (int c = 0; c < 3; ++c)
    {
        if (finalColor[c] < 0.0) finalColor[c] = 0.0;
        if (finalColor[c] > 1.0) finalColor[c] = 1.0;
        outColor[c] = (unsigned char)(finalColor[c] * 255.0 + 0.5);
    }
}

void draw_scene()
{
    double aspect = (double)WIDTH / (double)HEIGHT;
    double fovRad = fov * M_PI / 180.0;
    double halfHeight = tan(fovRad * 0.5);
    double halfWidth = aspect * halfHeight;

    double camOrigin[3] = { 0.0, 0.0, 0.0 };

  for(unsigned int x=0; x<WIDTH; x++)
  {
    glPointSize(2.0);  
    // Do not worry about this usage of OpenGL. This is here just so that we can draw the pixels to the screen,
    // after their R,G,B colors were determined by the ray tracer.
    glBegin(GL_POINTS);
    for(unsigned int y=0; y<HEIGHT; y++)
    {
        double u = ((double)x + 0.5) / (double)WIDTH;
        double v = ((double)y + 0.5) / (double)HEIGHT;

        double px = (2.0 * u - 1.0) * halfWidth;
        double py = (2.0 * v - 1.0) * halfHeight;
        double pz = -1.0; 

        double rayDir[3] = { px, py, pz };
        vec_normalize(rayDir);

        unsigned char rgb[3];
        trace_ray(camOrigin, rayDir, 0, rgb);

        plot_pixel(x, y, rgb[0], rgb[1], rgb[2]);
    }
    glEnd();
    glFlush();
  }
  printf("Ray tracing completed.\n"); 
  fflush(stdout);
}

void plot_pixel_display(int x, int y, unsigned char r, unsigned char g, unsigned char b)
{
  glColor3f(((float)r) / 255.0f, ((float)g) / 255.0f, ((float)b) / 255.0f);
  glVertex2i(x,y);
}

void plot_pixel_jpeg(int x, int y, unsigned char r, unsigned char g, unsigned char b)
{
  buffer[y][x][0] = r;
  buffer[y][x][1] = g;
  buffer[y][x][2] = b;
}

void plot_pixel(int x, int y, unsigned char r, unsigned char g, unsigned char b)
{
  plot_pixel_display(x,y,r,g,b);
  if(mode == MODE_JPEG)
    plot_pixel_jpeg(x,y,r,g,b);
}

void save_jpg()
{
  printf("Saving JPEG file: %s\n", filename);

  ImageIO img(WIDTH, HEIGHT, 3, &buffer[0][0][0]);
  if (img.save(filename, ImageIO::FORMAT_JPEG) != ImageIO::OK)
    printf("Error in saving\n");
  else 
    printf("File saved successfully\n");
}

void parse_check(const char *expected, char *found)
{
  if(strcasecmp(expected,found))
  {
    printf("Expected '%s ' found '%s '\n", expected, found);
    printf("Parsing error; abnormal program abortion.\n");
    exit(0);
  }
}

void parse_doubles(FILE* file, const char *check, double p[3])
{
  char str[100];
  fscanf(file,"%s",str);
  parse_check(check,str);
  fscanf(file,"%lf %lf %lf",&p[0],&p[1],&p[2]);
  printf("%s %lf %lf %lf\n",check,p[0],p[1],p[2]);
}

void parse_rad(FILE *file, double *r)
{
  char str[100];
  fscanf(file,"%s",str);
  parse_check("rad:",str);
  fscanf(file,"%lf",r);
  printf("rad: %f\n",*r);
}

void parse_shi(FILE *file, double *shi)
{
  char s[100];
  fscanf(file,"%s",s);
  parse_check("shi:",s);
  fscanf(file,"%lf",shi);
  printf("shi: %f\n",*shi);
}

int loadScene(char *argv)
{
  FILE * file = fopen(argv,"r");
  if (!file)
  {
    printf("Unable to open input file %s. Program exiting.\n", argv);
    exit(0);
  }

  int number_of_objects;
  char type[50];
  Triangle t;
  Sphere s;
  Light l;
  fscanf(file,"%i", &number_of_objects);

  printf("number of objects: %i\n",number_of_objects);

  parse_doubles(file,"amb:",ambient_light);

  for(int i=0; i<number_of_objects; i++)
  {
    fscanf(file,"%s\n",type);
    printf("%s\n",type);
    if(strcasecmp(type,"triangle")==0)
    {
      printf("found triangle\n");
      for(int j=0;j < 3;j++)
      {
        parse_doubles(file,"pos:",t.v[j].position);
        parse_doubles(file,"nor:",t.v[j].normal);
        parse_doubles(file,"dif:",t.v[j].color_diffuse);
        parse_doubles(file,"spe:",t.v[j].color_specular);
        parse_shi(file,&t.v[j].shininess);
      }

      if(num_triangles == MAX_TRIANGLES)
      {
        printf("too many triangles, you should increase MAX_TRIANGLES!\n");
        exit(0);
      }
      triangles[num_triangles++] = t;
    }
    else if(strcasecmp(type,"sphere")==0)
    {
      printf("found sphere\n");

      parse_doubles(file,"pos:",s.position);
      parse_rad(file,&s.radius);
      parse_doubles(file,"dif:",s.color_diffuse);
      parse_doubles(file,"spe:",s.color_specular);
      parse_shi(file,&s.shininess);

      if(num_spheres == MAX_SPHERES)
      {
        printf("too many spheres, you should increase MAX_SPHERES!\n");
        exit(0);
      }
      spheres[num_spheres++] = s;
    }
    else if(strcasecmp(type,"light")==0)
    {
      printf("found light\n");
      parse_doubles(file,"pos:",l.position);
      parse_doubles(file,"col:",l.color);

      if(num_lights == MAX_LIGHTS)
      {
        printf("too many lights, you should increase MAX_LIGHTS!\n");
        exit(0);
      }
      lights[num_lights++] = l;
    }
    else
    {
      printf("unknown type in scene description:\n%s\n",type);
      exit(0);
    }
  }
  return 0;
}

void display()
{
}

void init()
{
  glMatrixMode(GL_PROJECTION);
  glOrtho(0,WIDTH,0,HEIGHT,1,-1);
  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();

  glClearColor(0,0,0,0);
  glClear(GL_COLOR_BUFFER_BIT);
}

void idle()
{
  // Hack to make it only draw once.
  static int once=0;
  if(!once)
  {
    draw_scene();
    if(mode == MODE_JPEG)
      save_jpg();
  }
  once=1;
}

int main(int argc, char ** argv)
{
  if ((argc < 2) || (argc > 3))
  {  
    printf ("Usage: %s <input scenefile> [output jpegname]\n", argv[0]);
    exit(0);
  }
  if(argc == 3)
  {
    mode = MODE_JPEG;
    filename = argv[2];
  }
  else if(argc == 2)
    mode = MODE_DISPLAY;

  glutInit(&argc,argv);
  loadScene(argv[1]);

  glutInitDisplayMode(GLUT_RGBA | GLUT_SINGLE);
  glutInitWindowPosition(0,0);
  glutInitWindowSize(WIDTH,HEIGHT);
  int window = glutCreateWindow("Ray Tracer");
  #ifdef __APPLE__
    // This is needed on recent Mac OS X versions to correctly display the window.
    glutReshapeWindow(WIDTH - 1, HEIGHT - 1);
  #endif
  glutDisplayFunc(display);
  glutIdleFunc(idle);
  init();

  srand((unsigned int)time(NULL));

  glutMainLoop();
}

