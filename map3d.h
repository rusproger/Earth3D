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
      // Положение центра сцены (Земли)
      Vect3d scnmove;
      // Вращение относительно центра Земли
      Matrix3d locrotate;
      // Модельно-видовая матрица openGL
      Matrix4d matrix;
      // Точка относительно которой происходит движение/вращение сцены
      QPointF  MousePt;
      // Признаки движения/вращения сцены
      int m_rotateLocal;
      int m_moveScene;
      // Относительные координаты точки на сцене (приведены к размерам сцены)
      QPointF MouseProjection(const QPoint &pos);
      void ProjectionToVect3d(QPointF &proj, Vect3d &vec, GLdouble Radius);
      // Генерация кватерниона поворота
      void rotateToQuat4d(Vect3d *v0, Vect3d *v1, Quat4d *q);
      // Генерация матрицы поворота 3х3 из кватерниона
      void Quat4dToMatrix3d(Quat4d *q, Matrix3d *m);
      // Умножение кватернионов. Результат сохраняется в q1
      void MulQuat4d(Quat4d *q0, Quat4d *q1);
      // Умножение матриц 3х3. Результат сохраняется в m1
      void MulMatrix3d(Matrix3d *m0, Matrix3d *m1);
      // Добавление составляющих вращения в матрицу 4х4 из матрицы 3х3
      void Matrix4dRotateFromMatrix3d(Matrix4d *m4, Matrix3d *m3);
      // Добавление составляющих параллельного переноса в матрицу 4х4 из вектора
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
        // динамические объекты
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
  // Отобразить кнопки
  void showButtons(bool);
  // Обработчик нажатой Кнопки
  void mappingButtons(int value);
  // Установка начального положения карты
  void initPosition();

private:
  // Описание функций расширения OpenGL ES2
  // Данный функционал реализован в классе QGLFunction
  // Однако применить его из-за наличия ошибки выполнения при сборке в Release
  // QTBUG-27408 не представляется возможным

  #if defined(Q_OS_WIN)
  #    define GLAPIENTRY __stdcall
  #else
  #    define GLAPIENTRY
  #endif

  typedef void (GLAPIENTRY * GLCOMPRESSEDTEXIMAGE2D)(GLenum target, GLint level, GLenum internalformat, GLsizei width, GLsizei height, GLint border, GLsizei imageSize, const GLvoid *data);

  // Указатели на функции OpenGL ES2
  GLCOMPRESSEDTEXIMAGE2D glCompressedTexImage2D;

  // Информационные сообщения об ошибках при загрузке
  QString Message;
  // Массив точек для отрисовки
  QVector<GLdouble> Vertex;
  // Параметры привязки текстуры
  QVector<GLdouble> coordVertexSphere;
  QVector<GLdouble> normalVertexSphere;
  QVector<GLdouble> textureVertexSphere;
  // Текстура Земли
  GLuint earthText;


  QGridLayout *mapLayout;
  // Кнопка отображения меню
  QToolButton *menuButton;
  // Состояние флагов управления отображением
  QMap<ObjectsVisibleOnMap, bool> isVisibleObject;
  // Список QAction для отображению в Меню
  QMap<ObjectsVisibleOnMap, QAction*> visibleObjectsAction;
  // Контейнеры для сигналов от Меню и кнопок
  QSignalMapper *visibleMapper;
  // Список QPushButton для отображению в Меню
  QMap<ObjectsVisibleOnMap, QPushButton*> visibleObjectsButton;
  // Добавляет кнопку для управления отображением на карте
  void InsertButtons(const ObjectsVisibleOnMap key, bool value, QString name, QString text);
  // Добавляет текст в сообщение об ошибках загрузки карты (выводится при загрузке карты)
  void MESSAGE(QString text);

  // ФУНКЦИИ ЗАГРУЗКИ И ОТОБРАЖЕНИЯ ДАННЫХ НА КАРТЕ
  // Загрузка текстуры
  void loadTexture();
  // Расчет опорных точек наложения текстуры
  void createSphere(GLdouble xc, GLdouble yc, GLdouble zc, GLdouble r);
  // Загрузка параметров земли (текстура, опорные точки)
  void createEarth();
  // Отрисовка Земли (по загруженным текстуре и опорным точкам)
  void drawEarth();
  // Отобразить границу дня и ночи (линию терминатора)
  void drawSunLight();

};

#endif // MAP3D_H
