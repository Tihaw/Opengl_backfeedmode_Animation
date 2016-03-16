#include <windows.h>
#include <gl/gl.h>
#include <gl/glu.h>
#include <gl/glut.h>
#include <gl/glaux.h>
#include <math.h>
// 
// #pragma comment(lib, "OpenGl32.lib")
// #pragma comment(lib, "glu32.lib")
// #pragma comment(lib, "glaux.lib")

#pragma warning(disable : 4244)		// MIPS
#pragma warning(disable : 4136)		// X86
#pragma warning(disable : 4051)		// ALPHA

#define PI 3.1415f
#define Solid 1
#define Line 2
#define Point 3
#define DisAssembly 4
#define Assembly 5
#define Fast 6
#define Medium 7
#define Slow 8

static GLfloat xRot = 0.0f;
static GLfloat yRot = 0.0f;

static GLfloat zPin = 0.0f;
static GLfloat zBlock = 0.0f;

GLuint nPartsList;
GLuint curve_List,tool_List,BLOCK_LIST;
GLint interval_time = 100;
int	   MouseStartX;      
int    MouseStartY;    
int	   MouseX;      
int    MouseY;      
int iMode = Solid;
int iAssembly=0;
/************************************************************************/
/*   init params below																				*/
/************************************************************************/
// The number of control points for this curve
GLint nNumPoints = 4;

/*parm used for cordinates trans*/
GLint viewport[4];
GLdouble modelview[16];
GLdouble projection[16];  
GLdouble posX, posY, posZ;  

/*feedback buffers*/
GLfloat feedBuffer[1024];
GLfloat interpolationPts_DC[300];
GLdouble interpolationPts_OpenglC[300];
GLdouble interpolationPts_OpenglC_pair[300];
GLint interpolation_Counts = 0;
GLint interpolation_Counts_pair = 0;

/*controlling points*/
GLfloat ctrlPoints[4][3]= {{  -80.0f, 0.0f, -10},	// End Point
{ -100.0f, 80.0f, -30},	// Control Point
{  100.0f, -80.0f, 20},	// Control Point
{  80.0f, 0.0f, 30 }};	// End Point
GLfloat ctrlPoints_pair[4][3]=  {{  -80.0f, 40.0f, -10},	// End Point
{ -110.0f, 120.0f, -30},	// Control Point
{  120.0f, -40.0f, 20},	// Control Point
{  40.0f, 40.0f, 30 }};	// End Point

/*enum.....*/
enum
{
	first,
	second,
};


//定义一组控制点的存储空间 
GLfloat points[4][4][3]; 

//定义一个指向NURBS曲面对象的指针 
GLUnurbsObj *theNurb; 
GLfloat color[4][3]={ 
	{1.0,0.0,0.0},{0.0,1.0,0.0},{0.0,0.0,1.0},{1.0,1.0,1.0}}; 

	void screen2openglCor (GLfloat *interpolationPts_DC, GLdouble *interpolationPts_OpenglC)
	{
		GLint count = 0;
		while (count < 300 )
		{
			gluUnProject(	interpolationPts_DC[count], 
				interpolationPts_DC[count+1], 
				interpolationPts_DC[count+2],
				modelview, projection, viewport, 
				&interpolationPts_OpenglC[count], 
				&interpolationPts_OpenglC[count+1], 
				&interpolationPts_OpenglC[count+2]);  
			count += 3;
		}
		return;
	}

	/*only parse for line segment,
	ignore point, polygon, bitmap...*/
	void parseFeedback2InterpolationPts(GLint size, GLfloat *feedBuffer)
	{
		GLint count = 0;
		GLfloat *p_interpolationPts_DC = interpolationPts_DC;
		while (count < size )
		{
			/*0 7 14 21....693 are makers*/
			if (count % 7 == 0)
			{
				count ++;
				*p_interpolationPts_DC = feedBuffer[count];
				*(p_interpolationPts_DC+1) = feedBuffer[count+1];
				*(p_interpolationPts_DC+2) = feedBuffer[count+2];
				p_interpolationPts_DC += 3;
				continue;
			}
			count ++;
		}
	}

	// This function is used to superimpose the control points over the curve
	void DrawPoints(GLint curve_ID)
	{
		int i;	// Counting variable
		GLfloat* p_Pts;
		if (curve_ID == first)
		{
			p_Pts = &ctrlPoints[0][0];
		}
		else
		{
			p_Pts = &ctrlPoints_pair[0][0];
		}

		glColor4f(1.0,1.0,0,1);
		// Set point size larger to make more visible
		glPointSize(5.0f);
		
		// Loop through all control points for this example
		glBegin(GL_POINTS);
		for(i = 0; i < nNumPoints; i++)
			glVertex2fv(p_Pts + i * 3);
		glEnd();

		//DRAW control Polygon
		glColor4f(0.3, 0.3, 0.3,0.1);
		glBegin(GL_LINE_STRIP);
		for(i = 0; i < nNumPoints; i++)
			glVertex2fv(p_Pts + i * 3);
		glEnd();
	}

//计算向量
void CalculateNormal(GLfloat *dVertex1, GLfloat *dVertex2, GLfloat *dVertex3, GLfloat *dNormal)
{
    GLfloat dVector1[3],dVector2[3];
	dVector1[0]=dVertex2[0]-dVertex1[0];
	dVector1[1]=dVertex2[1]-dVertex1[1];
	dVector1[2]=dVertex2[2]-dVertex1[2];
	dVector2[0]=dVertex3[0]-dVertex1[0];
	dVector2[1]=dVertex3[1]-dVertex1[1];
	dVector2[2]=dVertex3[2]-dVertex1[2];

	dNormal[0]=dVector1[1]*dVector2[2]-dVector1[2]*dVector2[1];
	dNormal[1]=dVector1[2]*dVector2[0]-dVector1[0]*dVector2[2];
	dNormal[2]=dVector1[0]*dVector2[1]-dVector1[1]*dVector2[0];

	GLfloat dNormalLength=sqrt(dNormal[0]*dNormal[0]+dNormal[1]*dNormal[1]+dNormal[2]*dNormal[2]);
	if (dNormalLength!=0.0)
	{
		dNormal[0]=dNormal[0]/dNormalLength;
		dNormal[1]=dNormal[1]/dNormalLength;
		dNormal[2]=dNormal[2]/dNormalLength;
	}
	else
	{
		dNormal[0]=0.0;
		dNormal[1]=0.0;
		dNormal[2]=1.0;
	}

}
void TimerFunc(int value)
{
    glutPostRedisplay();
    glutTimerFunc(interval_time, TimerFunc, 1);
}

//快捷菜单
void ProcessMenu(int value)
{
	switch(value)
	{
		case 1:
			iMode = Solid;	
			break;
		case 2:
			iMode = Line;	
			break;
		case 3:
			iMode = Point;	
			break;
		case 4:
			iAssembly = DisAssembly;
			break;
		case 5:
			iAssembly = Assembly;
			break;
		case 6:
			interval_time = 50;
			break;
		case 7:
			interval_time = 100;
			break;
		case 8:
			interval_time = 500;
			break;
		default:
			break;
	}
	glutPostRedisplay();
}


/*draw the bezier curve*/
/*argument - different curves*/
void drawCurve (GLint curve_ID)
{
	int i=0;
	glColor4f(1,1,0,1);
/*
	//a maker
	if (mode == GL_FEEDBACK)
		glPassThrough (1.0);*/

	//draw curve
	// Sets up the bezier
	// This actually only needs to be called once and could go in
	// the setup function
	if (curve_ID == first)
	{
		glMap1f(GL_MAP1_VERTEX_3,	// Type of data generated
			0.0f,						// Lower u range
			100.0f,						// Upper u range
			3,							// Distance between points in the data
			nNumPoints,					// number of control points
			&ctrlPoints[0][0]);			// array of control points
	}
	else
	{
		glMap1f(GL_MAP1_VERTEX_3,	// Type of data generated
			0.0f,						// Lower u range
			100.0f,						// Upper u range
			3,							// Distance between points in the data
			nNumPoints,					// number of control points
			&ctrlPoints_pair[0][0]);			// array of control points
	}

	// Enable the evaluator
	glEnable(GL_MAP1_VERTEX_3);

	// Use a line strip to "connect-the-dots"
/*
	glBegin(GL_LINE_STRIP);//画折线
	for(i = 0; i <= 100; i++)
	{
		// Evaluate the curve at this point
		glEvalCoord1f((GLfloat) i);//计算不同参数下的曲线上的值 
	}
	glEnd();*/

	// Map a grid of 100 points from 0 to 100
	glMapGrid1d(100,0.0,100.0);

	// Evaluate the grid, using lines
	glEvalMesh1(GL_LINE,0,100);

/*
	//a maker 
	if (mode == GL_FEEDBACK)
		glPassThrough (2.0);* /*/
}

/*draw the machine tool*/
void drawTool(void)
{
	//glRotatef(90,1,0,0);
	GLfloat x,y,angle; 
	GLfloat step = (PI/50.0f);
	GLfloat diameter = 5.0f;
	GLfloat dVertex1[3]={0.0};
	GLfloat dVertex2[3]={0.0};
	GLfloat dVertex3[3]={0.0};
	GLfloat dNormal[3]={0.0f,0.0f,1.0f};

	glColor3f(0.5f,0.5f,0.5f);//制定颜色-蓝
	glFrontFace(GL_CCW);
	glBegin(GL_TRIANGLE_FAN);
	glNormal3fv(dNormal);
	glVertex3f(0.0f,0.0f,50.0f);
	for(angle=0.0f;angle<PI*2.0f;angle+=step)
	{
		x=diameter*(float)cos(angle);
		y=diameter*(float)sin(angle);
		glVertex3f(x,y,50.0f);
	}
	glVertex3f(diameter,0.0f,50.0f);
	glEnd();//销钉顶面

	glBegin(GL_QUAD_STRIP);
	for(angle=0.0f;angle<PI*2.0f;angle+=step)
	{
		x=diameter*(float)cos(angle);
		y=diameter*(float)sin(angle);
		dVertex1[0]=diameter*(float)cos(angle-step);
		dVertex1[1]=diameter*(float)sin(angle-step);
		dVertex1[2]=50.0;
		dVertex2[0]=x; dVertex2[1]=y; dVertex2[2]=0.0f;
		dVertex3[0]=x; dVertex3[1]=y; dVertex3[2]=50.0f;
		CalculateNormal(dVertex1,dVertex2,dVertex3,dNormal);
		glNormal3fv(dNormal);
		glVertex3f(x,y,50.0f);
		glVertex3f(x,y,10.0f);
	}
	glVertex3f(diameter,0.0f,50.0f);
	glVertex3f(diameter,0.0f,0.0f);
	glEnd();//销钉侧面

	glTranslatef(0,0,10);
	glutSolidSphere(5,10,10);
}

/*parse curve into line segment or pts*/
void parseCurve() 
{
	GLint size;

	//set the values
	glGetDoublev(GL_MODELVIEW_MATRIX, modelview);  
	glGetDoublev(GL_PROJECTION_MATRIX, projection);  
	glGetIntegerv(GL_VIEWPORT, viewport);  

	/* the normal | default mode, drawing*/
	//  drawGeometry ( GL_RENDER);

	/*that works for feedback*/
	glFeedbackBuffer (1024, GL_3D, feedBuffer);
	(void) glRenderMode (GL_FEEDBACK);
	/*what is feedback 'draws' here :) */
	drawCurve ( first);
	/*process the feedback value*/
	/*return to normal drawing mode*/
	size = glRenderMode (GL_RENDER);
	parseFeedback2InterpolationPts(size, feedBuffer);
	screen2openglCor(interpolationPts_DC, interpolationPts_OpenglC);

	/************************************************************************/
	/*  parse the second curve                                                                    */
	/************************************************************************/
	/*that works for feedback*/
	glFeedbackBuffer (1024, GL_3D, feedBuffer);
	(void) glRenderMode (GL_FEEDBACK);
	/*what is feedback 'draws' here :) */
	drawCurve ( second);
	/*process the feedback value*/
	/*return to normal drawing mode*/
	size = glRenderMode (GL_RENDER);
	parseFeedback2InterpolationPts(size, feedBuffer);
	screen2openglCor(interpolationPts_DC, interpolationPts_OpenglC_pair);
}

void RenderScene(void)
{
	GLfloat cur_Pos_x = 0, cur_Pos_y = 0, cur_Pos_z = 0;
	parseCurve();
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	if(iMode==Solid)
		glPolygonMode(GL_FRONT_AND_BACK,GL_FILL);
    if(iMode==Line)
		glPolygonMode(GL_FRONT_AND_BACK,GL_LINE);
	if(iMode==Point)
		glPolygonMode(GL_FRONT_AND_BACK,GL_POINT);
	if(iAssembly==DisAssembly)
	{
		if (interpolation_Counts < 300)
		{
			cur_Pos_x = interpolationPts_OpenglC[interpolation_Counts];
			cur_Pos_y = interpolationPts_OpenglC[interpolation_Counts+1];
			cur_Pos_z = interpolationPts_OpenglC[interpolation_Counts+2];
			interpolation_Counts += 3;
		}	
		else
		{
			if (interpolation_Counts_pair <300)
			{
				cur_Pos_x = interpolationPts_OpenglC_pair[interpolation_Counts_pair];
				cur_Pos_y = interpolationPts_OpenglC_pair[interpolation_Counts_pair+1];
				cur_Pos_z = interpolationPts_OpenglC_pair[interpolation_Counts_pair+2];
				interpolation_Counts_pair += 3;
			}
		}
	}
	if (iAssembly==Assembly)
	{
		interpolation_Counts = 0;
		interpolation_Counts_pair = 0;
		iAssembly = DisAssembly;
	}
	
	// Rotate about x and y axes
	glRotatef(xRot, 1.0f, 0.0f, 0.0f);
	glRotatef(yRot, 0.0f, 0.0f, 1.0f);

	// Render just the Thread of the nut

	glRotatef(-55.0f, 1.0f, 0.0f, 0.0f);
	glRotatef(-45.0f,0.0f,0.0f,1.0f);
	glPushMatrix();
    glCallList(curve_List);
	glTranslatef(cur_Pos_x, cur_Pos_y, cur_Pos_z-5);
	glCallList(tool_List);
	glPopMatrix();


	glColor4f(0, 1, 1,0.1);
	glBegin(GL_LINE_STRIP);
	glVertex3f(interpolationPts_OpenglC[297],
		interpolationPts_OpenglC[298],
		interpolationPts_OpenglC[299]);
	glVertex3f(interpolationPts_OpenglC[297],
		interpolationPts_OpenglC[298],
		interpolationPts_OpenglC[299]+2);
	glVertex3f(interpolationPts_OpenglC[297],
		interpolationPts_OpenglC[298],
		interpolationPts_OpenglC[299]+20);
	glVertex3f(interpolationPts_OpenglC_pair[0],
		interpolationPts_OpenglC_pair[1],
		interpolationPts_OpenglC_pair[2]+20);
	glVertex3f(interpolationPts_OpenglC_pair[0],
		interpolationPts_OpenglC_pair[1],
		interpolationPts_OpenglC_pair[2]);
	glEnd();

	// Swap buffers
	glutSwapBuffers();
}

void SetupRC()
{
    GLfloat  ambientLight[] = {0.4f, 0.4f, 0.4f, 1.0f};
    GLfloat  diffuseLight[] = {0.7f, 0.7f, 0.7f, 1.0f};
    GLfloat  specular[] = { 0.9f, 0.9f, 0.9f, 1.0f};
    GLfloat  lightPos[] = { -50.0f, 200.0f, 200.0f, 1.0f};
    GLfloat  specref[] =  { 0.6f, 0.6f, 0.6f, 1.0f};

    glEnable(GL_DEPTH_TEST);    // Hidden surface removal
    glEnable(GL_CULL_FACE);     // Do not calculate inside of solid object
    glFrontFace(GL_CCW);
    
    // Enable lighting
    glEnable(GL_LIGHTING);

    glLightModelfv(GL_LIGHT_MODEL_AMBIENT,ambientLight);
    glLightfv(GL_LIGHT0,GL_AMBIENT,ambientLight);
    glLightfv(GL_LIGHT0,GL_DIFFUSE,diffuseLight);
    glLightfv(GL_LIGHT0,GL_SPECULAR,specular);

    // Position and turn on the light
    glLightfv(GL_LIGHT0,GL_POSITION,lightPos);
    glEnable(GL_LIGHT0);

    // Enable color tracking
    glEnable(GL_COLOR_MATERIAL);
    
    // Set Material properties to follow glColor values
    glColorMaterial(GL_FRONT, GL_AMBIENT_AND_DIFFUSE);

    // All materials hereafter have full specular reflectivity
    // with a moderate shine
    glMaterialfv(GL_FRONT, GL_SPECULAR,specref);
    glMateriali(GL_FRONT,GL_SHININESS,64);

    // Black background
    glClearColor(0.0f, 0.0f, 0.0f, 1.0f);

    // Set three display list names
    curve_List = nPartsList;//底座表
    tool_List = nPartsList+1;//销钉表
    BLOCK_LIST = nPartsList+2;//定位块表

	glNewList(curve_List,GL_COMPILE_AND_EXECUTE);
	drawCurve(first);
	DrawPoints(first);
	drawCurve(second);
	DrawPoints(second);
	glEndList();

	glNewList(tool_List,GL_COMPILE_AND_EXECUTE);
       drawTool();//绘制销钉
    glEndList();

    }

///////////////////////////////////////////////////////////////////////////////
// Process arrow keys
void SpecialKeys(int key, int x, int y)
    {
    if(key == GLUT_KEY_UP)
        xRot-= 5.0f;

    if(key == GLUT_KEY_DOWN)
        xRot += 5.0f;

    if(key == GLUT_KEY_LEFT)
        yRot -= 5.0f;

    if(key == GLUT_KEY_RIGHT)
        yRot += 5.0f;

    if(key > 356.0f)
        xRot = 0.0f;

    if(key < -1.0f)
        xRot = 355.0f;

    if(key > 356.0f)
        yRot = 0.0f;

    if(key < -1.0f)
        yRot = 355.0f;

    // Refresh the Window
    glutPostRedisplay();
    }


void ChangeSize(int w, int h)
    {
    GLfloat nRange = 100.0f;

    if(h == 0)
        h = 1;

    // Set Viewport to window dimensions
    glViewport(0, 0, w, h);

    // Reset coordinate system
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();

    // Establish clipping volume (left, right, bottom, top, near, far)
    if (w <= h) 
        glOrtho (-nRange, nRange, -nRange*h/w, nRange*h/w, -nRange*2.0f, nRange*2.0f);
    else 
        glOrtho (-nRange*w/h, nRange*w/h, -nRange, nRange, -nRange*2.0f, nRange*2.0f);

    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    }

void Motion(int x,int y)
{
    MouseX = x;
    MouseY = y;
}

void Idle(void)
{
	yRot+=0.01*(MouseX-MouseStartX);
	xRot+=0.01*(MouseY-MouseStartY);
	glutPostRedisplay();
}

void Mouse(int button,int state,int x,int y)
{
    MouseStartX = MouseX = x;
    MouseStartY = MouseY = y;

    if (state == GLUT_DOWN)//左键按下
    {
         glutIdleFunc(Idle);
	}

}

int main(int argc, char* argv[])
{
    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
	glutInitWindowSize(500,450);
	glutInitWindowPosition(250,100);
    glutCreateWindow("刀具沿Bezier曲线运动，回程轨迹为青色");

	glutCreateMenu(ProcessMenu);
	glutAddMenuEntry("Solid",1);
	glutAddMenuEntry("Lines",2);
	//glutAddMenuEntry("Points",3);
	glutAddMenuEntry("Go",4);
	glutAddMenuEntry("Replay",5);
	glutAddMenuEntry("Fast",6);
	glutAddMenuEntry("Medium",7);
	glutAddMenuEntry("Slow",8);
	glutAttachMenu(GLUT_RIGHT_BUTTON);
	
    glutReshapeFunc(ChangeSize);
	glutMouseFunc(Mouse);
    glutMotionFunc(Motion);
    glutSpecialFunc(SpecialKeys);
    glutDisplayFunc(RenderScene);
	glutTimerFunc(5, TimerFunc, 1);
    // Create three display list names
    nPartsList = glGenLists(3);//建立3个显示表

    SetupRC();
	glutMainLoop();

    // Delete the display lists
    glDeleteLists(nPartsList, 3);
    return 0;
    }
