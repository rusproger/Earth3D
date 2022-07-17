#ifndef MAP3D_H
#define MAP3D_H

#include <QtOpenGL>


class M3D_CAMERA: public QGLWidget
{

    typedef struct
    {
      GLdouble x,y,z,w;
    }Quat4d;

    typedef struct
    {
      GLdouble x,y,z;
    }Vect3d;

    typedef  struct
    {
      GLdouble xx,xy,xz,
               yx,yy,yz,
               zx,zy,zz;
    } Matrix3d;
    typedef  struct
    {
        GLdouble xx,xy,xz,xw,
                 yx,yy,yz,yw,
                 zx,zy,zz,zw,
                 tx,ty,tz,tw;
    }Matrix4d;
protected:
      void wheelEvent(QWheelEvent *event);
      void mouseMoveEvent(QMouseEvent *event);
      void mousePressEvent(QMouseEvent *event);
      void mouseReleaseEvent(QMouseEvent *event);

      void resizeGL(int w, int h);
      void setModelViewMatrix();
private:
      // ��������� ������ ����� (�����)
      Vect3d scnmove;
      // �������� ������������ ������ �����
      Matrix3d locrotate;
      // ��������-������� ������� openGL
      Matrix4d matrix;
      // ����� ������������ ������� ���������� ��������/�������� �����
      QPointF  MousePt;
      // �������� ��������/�������� �����
      int m_rotateLocal;
      int m_moveScene;
      // ������������� ���������� ����� �� ����� (��������� � �������� �����)
      QPointF MouseProjection(const QPoint &pos);
      void ProjectionToVect3d(QPointF &proj, Vect3d &vec, GLdouble Radius);
      // ��������� ����������� ��������
      void rotateToQuat4d(Vect3d *v0, Vect3d *v1, Quat4d *q);
      // ��������� ������� �������� 3�3 �� �����������
      void Quat4dToMatrix3d(Quat4d *q, Matrix3d *m);
      // ��������� ������������. ��������� ����������� � q1
      void MulQuat4d(Quat4d *q0, Quat4d *q1);
      // ��������� ������ 3�3. ��������� ����������� � m1
      void MulMatrix3d(Matrix3d *m0, Matrix3d *m1);
      // ���������� ������������ �������� � ������� 4�4 �� ������� 3�3
      void Matrix4dRotateFromMatrix3d(Matrix4d *m4, Matrix3d *m3);
      // ���������� ������������ ������������� �������� � ������� 4�4 �� �������
      void Matrix4dTranslateFromVect3d(Matrix4d *m4, Vect3d *v3);
      void moveScene(QMouseEvent *event);
      void rotateLocal(QMouseEvent *event);
public:
    M3D_CAMERA(QWidget *parent);
    ~M3D_CAMERA();

    void initCamera(GLdouble scnx, GLdouble scny, GLdouble lat, GLdouble lon, GLdouble alt);

};

class CMap3d : public M3D_CAMERA
{
  Q_OBJECT

    enum ObjectsVisibleOnMap {
        // ������������ �������
        show_light,
        default_pos
    };

public:
  CMap3d(QWidget *parent = 0);
  ~CMap3d();

protected:
  void initializeGL();
  void paintGL();

private slots:
  // ���������� ������
  void showButtons(bool);
  // ���������� ������� ������
  void mappingButtons(int value);
  // ��������� ���������� ��������� �����
  void initPosition();

private:
  // �������� ������� ���������� OpenGL ES2
  // ������ ���������� ���������� � ������ QGLFunction
  // ������ ��������� ��� ��-�� ������� ������ ���������� ��� ������ � Release
  // QTBUG-27408 �� �������������� ���������

  #if defined(Q_OS_WIN)
  #    define GLAPIENTRY __stdcall
  #else
  #    define GLAPIENTRY
  #endif

  typedef void (GLAPIENTRY * GLCOMPRESSEDTEXIMAGE2D)(GLenum target, GLint level, GLenum internalformat, GLsizei width, GLsizei height, GLint border, GLsizei imageSize, const GLvoid *data);

  // ��������� �� ������� OpenGL ES2
  GLCOMPRESSEDTEXIMAGE2D glCompressedTexImage2D;

  // �������������� ��������� �� ������� ��� ��������
  QString Message;
  // ������ ����� ��� ���������
  QVector<GLdouble> Vertex;
  // ��������� �������� ��������
  QVector<GLdouble> coordVertexSphere;
  QVector<GLdouble> normalVertexSphere;
  QVector<GLdouble> textureVertexSphere;
  // �������� �����
  GLuint earthText;


  QGridLayout *mapLayout;
  // ������ ����������� ����
  QToolButton *menuButton;
  // ��������� ������ ���������� ������������
  QMap<ObjectsVisibleOnMap, bool> isVisibleObject;
  // ������ QAction ��� ����������� � ����
  QMap<ObjectsVisibleOnMap, QAction*> visibleObjectsAction;
  // ���������� ��� �������� �� ���� � ������
  QSignalMapper *visibleMapper;
  // ������ QPushButton ��� ����������� � ����
  QMap<ObjectsVisibleOnMap, QPushButton*> visibleObjectsButton;
  // ��������� ������ ��� ���������� ������������ �� �����
  void InsertButtons(const ObjectsVisibleOnMap key, bool value, QString name, QString text);
  // ��������� ����� � ��������� �� ������� �������� ����� (��������� ��� �������� �����)
  void MESSAGE(QString text);

  // ������� �������� � ����������� ������ �� �����
  // �������� ��������
  void loadTexture();
  // ������ ������� ����� ��������� ��������
  void createSphere(GLdouble xc, GLdouble yc, GLdouble zc, GLdouble r);
  // �������� ���������� ����� (��������, ������� �����)
  void createEarth();
  // ��������� ����� (�� ����������� �������� � ������� ������)
  void drawEarth();
  // ���������� ������� ��� � ���� (����� �����������)
  void drawSunLight();

};

#endif // MAP3D_H
