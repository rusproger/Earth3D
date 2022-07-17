#include <QCoreApplication>
#include <QTransform>
#include <QtGui/QInputEvent>
#include <QImage>
#include<ctime>

#ifndef MAP3D_H
  #include "./map3d.h"
#endif

#define MIN_VALUE       1e-12
#define DECL_PATH_ELEM  1500
#ifndef M_2PI
    #define M_2PI        (2.0*M_PI)
#endif
#define M_DEG_TO_RAD  (M_PI/180.0)
#define M_RAD_TO_DEG  (180.0/M_PI)

// Подставлены координаты центра Москвы
// https://ru.wikipedia.org/wiki/%D0%93%D0%B5%D0%BE%D0%B3%D1%80%D0%B0%D1%84%D0%B8%D1%8F_%D0%9C%D0%BE%D1%81%D0%BA%D0%B2%D1%8B
// В сентябре 2014 года в честь 867-летия Москвы специалисты Московского государственного университета
// геодезии и картографии (МИИГАиК) определили координаты географического центра Москвы (по центру масс всех частей города,
// с учётом Зеленограда): 55,558741° с. ш., 37,378847° в. д.
//
#define SHIRMPSK (55.558741*M_DEG_TO_RAD)
#define DOLGMPSK (37.378847*M_DEG_TO_RAD)

// Разбиение глобуса
#define LG_SLICE_CNT 64


// Размер точки и ширина линии
#define POINT_SIZE   5.0f
#define LINE_WIDTH   1.0f


// Класс 3D камеры
// Реализовано движение центра Земли в прямоугольной системе координат
// и вращение Земли вокруг центра
// Использована статья nehe "ArcBall Rotation" lesson 48
// http://nehe.gamedev.net/tutorial/arcball_rotation/19003/
// http://rainwarrior.ca/dragon/arcball.html

#define LG_FOVY      15.0
#define LG_NEAR_DIST 0.0001
#define LG_FAR_DIST  100.0
#define LG_CAM_SPEED 0.03
#define LG_RADIUS    1.0


M3D_CAMERA::M3D_CAMERA(QWidget *parent)
    : QGLWidget(parent), m_rotateLocal(0), m_moveScene(0)
{
    initCamera(0.0, 0.0, 0.0, 0.0, 1.0);
}

M3D_CAMERA::~M3D_CAMERA()
{

}


// Генерация кватерниона поворота
void M3D_CAMERA::rotateToQuat4d(Vect3d *v0, Vect3d *v1, Quat4d *q)
{
    GLdouble n;

    q->x = q->y = q->z = 0.0;
    q->w = 1.0;

    n = sqrt((v0->x*v0->x + v0->y*v0->y + v0->z*v0->z)*(v1->x*v1->x + v1->y*v1->y + v1->z*v1->z));
    if (n > MIN_VALUE)
    {
        // Рассматриваем частный случай, угол между векторами строго меньше 180 град
        q->x  = (v0->y * v1->z - v1->y * v0->z) / n;
        q->y  = (v1->x * v0->z - v0->x * v1->z) / n;
        q->z  = (v0->x * v1->y - v1->x * v0->y) / n;
        q->w  = (v0->x*v1->x + v0->y*v1->y + v0->z*v1->z) / n + 1.0;
    }
}

// Умножение кватернионов. Результат сохраняется в q1
void M3D_CAMERA::MulQuat4d(Quat4d *q0, Quat4d *q1)
{
    GLdouble A, B, C, D, E, F, G, H;
    Quat4d r;

    A = (q0->w + q0->x) * (q1->w + q1->x);
    B = (q0->z - q0->y) * (q1->y - q1->z);
    C = (q0->x - q0->w) * (q1->y + q1->z);
    D = (q0->y + q0->z) * (q1->x - q1->w);
    E = (q0->x + q0->z) * (q1->x + q1->y);
    F = (q0->x - q0->z) * (q1->x - q1->y);
    G = (q0->w + q0->y) * (q1->w - q1->z);
    H = (q0->w - q0->y) * (q1->w + q1->z);

    r.w = B + (-E - F + G + H) * 0.5;
    r.x = A - ( E + F + G + H) * 0.5;
    r.y =-C + ( E - F + G - H) * 0.5;
    r.z =-D + ( E - F - G + H) * 0.5;

    *q1 = r;
}

// Генерация матрицы поворота 3х3 из кватерниона
void M3D_CAMERA::Quat4dToMatrix3d(Quat4d *q, Matrix3d *m)
{
    GLdouble n, s;
    GLdouble xs, ys, zs;
    GLdouble wx, wy, wz;
    GLdouble xx, xy, xz;
    GLdouble yy, yz, zz;


    n = (q->x * q->x) + (q->y * q->y) + (q->z * q->z) + (q->w * q->w);
    s = (n > 0.0) ? (2.0 / n) : 0.0;

    xs = q->x * s;  ys = q->y * s;  zs = q->z * s;
    wx = q->w * xs; wy = q->w * ys; wz = q->w * zs;
    xx = q->x * xs; xy = q->x * ys; xz = q->x * zs;
    yy = q->y * ys; yz = q->y * zs; zz = q->z * zs;

    m->xx = 1.0f - (yy + zz); m->yx =         xy - wz;  m->zx =         xz + wy;
    m->xy =         xy + wz;  m->yy = 1.0f - (xx + zz); m->zy =         yz - wx;
    m->xz =         xz - wy;  m->yz =         yz + wx;  m->zz = 1.0f - (xx + yy);
}
// Умножение матриц 3х3. Результат сохраняется в m1
void M3D_CAMERA::MulMatrix3d(Matrix3d *m0, Matrix3d *m1)
{
    Matrix3d r;


    r.xx = (m0->xx * m1->xx) + (m0->yx * m1->xy) + (m0->zx * m1->xz);
    r.yx = (m0->xx * m1->yx) + (m0->yx * m1->yy) + (m0->zx * m1->yz);
    r.zx = (m0->xx * m1->zx) + (m0->yx * m1->zy) + (m0->zx * m1->zz);

    r.xy = (m0->xy * m1->xx) + (m0->yy * m1->xy) + (m0->zy * m1->xz);
    r.yy = (m0->xy * m1->yx) + (m0->yy * m1->yy) + (m0->zy * m1->yz);
    r.zy = (m0->xy * m1->zx) + (m0->yy * m1->zy) + (m0->zy * m1->zz);

    r.xz = (m0->xz * m1->xx) + (m0->yz * m1->xy) + (m0->zz * m1->xz);
    r.yz = (m0->xz * m1->yx) + (m0->yz * m1->yy) + (m0->zz * m1->yz);
    r.zz = (m0->xz * m1->zx) + (m0->yz * m1->zy) + (m0->zz * m1->zz);

    *m1 = r;

}
// Добавление составляющих вращения в матрицу 4х4 из матрицы 3х3
void M3D_CAMERA::Matrix4dRotateFromMatrix3d(Matrix4d *m4, Matrix3d *m3)
{
    GLdouble scale;
    // this is a simple svd.
    // Not complete but fast and reasonable.
    scale = sqrt(
            ( (m3->xx * m3->xx) + (m3->xy * m3->xy) + (m3->xz * m3->xz) +
              (m3->yx * m3->yx) + (m3->yy * m3->yy) + (m3->yz * m3->yz) +
              (m3->zx * m3->zx) + (m3->zy * m3->zy) + (m3->zz * m3->zz) ) / 3.0 );

    m4->xx = m3->xx * scale; m4->xy = m3->xy * scale; m4->xz = m3->xz * scale;
    m4->yx = m3->yx * scale; m4->yy = m3->yy * scale; m4->yz = m3->yz * scale;
    m4->zx = m3->zx * scale; m4->zy = m3->zy * scale; m4->zz = m3->zz * scale;
}

// Добавление составляющих параллельного переноса в матрицу 4х4 из вектора
void M3D_CAMERA::Matrix4dTranslateFromVect3d(Matrix4d *m4, Vect3d *v3)
{
    m4->tx = - v3->x;
    m4->ty = - v3->y;
    m4->tz = - v3->z;
}

void M3D_CAMERA::initCamera(GLdouble scnx, GLdouble scny, GLdouble lat, GLdouble lon, GLdouble alt)
{
    Quat4d Qlat, Qlon;

    alt = qBound(LG_NEAR_DIST, alt, LG_FAR_DIST - 2.0);

    // Инициализация вектора положения центра Земли (ось Z на нас поэтому "-")
    scnmove.x  = scnx;
    scnmove.y  = scny;
    scnmove.z  = 1.0 + alt;

    // Поворот Земли вокрух центра
    Qlat.x = sin(lat/2.0); Qlat.y = 0.0;           Qlat.z = 0.0; Qlat.w = cos(lat/2.0);
    Qlon.x = 0.0;          Qlon.y = -sin(lon/2.0); Qlon.z = 0.0; Qlon.w = cos(lon/2.0);
    MulQuat4d(&Qlat, &Qlon);
    Quat4dToMatrix3d(&Qlon, &locrotate);

    // Инициализация матрицы
    matrix.xx = 1.0; matrix.xy = 0.0; matrix.xz = 0.0; matrix.tx = 0.0;
    matrix.yx = 0.0; matrix.yy = 1.0; matrix.yz = 0.0; matrix.ty = 0.0;
    matrix.zx = 0.0; matrix.zy = 0.0; matrix.zz = 1.0; matrix.tz = 0.0;
    matrix.xw = 0.0; matrix.yw = 0.0; matrix.zw = 0.0; matrix.tw = 1.0;

    // Применение поворота и сдвига начала координат к матрице
    Matrix4dTranslateFromVect3d(&matrix,&scnmove);
    Matrix4dRotateFromMatrix3d (&matrix,&locrotate);
}

void M3D_CAMERA::mousePressEvent(QMouseEvent *event)
{
      switch(event->button())
      {
      case Qt::LeftButton:
          m_rotateLocal = 1;
          break;
      case Qt::RightButton:
          m_moveScene = 1;
          break;
      default:
          break;
      }
      MousePt = MouseProjection(event->pos());
      setCursor(Qt::ClosedHandCursor);
}

// Относительные координаты точки на сцене
QPointF M3D_CAMERA::MouseProjection(const QPoint &pos)
{
    GLdouble ratio = (GLdouble)width() / height(),
    h  = tan(LG_FOVY * M_DEG_TO_RAD),
    w  = h * ratio;

    return QPointF(w * (2.0 * pos.x() / width() - 1.0),
                   h * (1.0 - 2.0 * pos.y() / height()));
}

void M3D_CAMERA::ProjectionToVect3d(QPointF &proj, Vect3d &vec, GLdouble Radius)
{
    // Решение задачи о поиске точки на сфере (x,y,z) c центром (xc,yc,zc) и радиусом R по относительным экранным координатам курсора (xp,yp):
    //
    // (xp*(zc-z)+xc)^2 + (yp*(zc-z)+yc)^2 + z^2 = R^2


     GLdouble A  = proj.x()*proj.x() + proj.y()*proj.y() + 1.0,
              B  = scnmove.z *(proj.x()*proj.x() + proj.y()*proj.y()) + scnmove.x*proj.x() + scnmove.y*proj.y(),
              C  = scnmove.z*scnmove.z*(proj.x()*proj.x() + proj.y()*proj.y())
                    + 2.0*scnmove.z*(scnmove.x*proj.x() + scnmove.y*proj.y())
                    + scnmove.x*scnmove.x + scnmove.y*scnmove.y - Radius*Radius,
              D  = B*B - A*C;

    if (D > 0.0)
    {
        vec.z = (B + sqrt(D))/A;
        vec.x = proj.x() * (scnmove.z - vec.z) + scnmove.x;
        vec.y = proj.y() * (scnmove.z - vec.z) + scnmove.y;
    }
    else
    {
        // Решение задачи о поиске вектора проходящего через точку на крае видимого диска сферы радиусом R и центр сферы (xc,yc,zc)
        // по относительным экранным координатам курсора (xp,yp):
        // Искомый вектор лежит на поверхности конуса. Используем св-ва скалярного произведения:
        // (xp*(zc-z)+xc)*xc + (yp*(zc-z)+yc)*yc + z*zc = R*sqrt((xp*(zc-z)+xc)^2 + (yp*(zc-z)+yc)^2 + z^2)
        // Формулы получены с помощью ресурса Wolfram Alfa:
        // simplify ((x*(C-z)+A)*A + (y*(C-z)+B)*B + z*C)^2-R*R*((x*(C-z)+A)^2 + (y*(C-z)+B)^2 + z^2)=0
        A = scnmove.x*scnmove.x*proj.x()*proj.x()
                + 2*scnmove.x*scnmove.y*proj.x()*proj.y()
                - 2*scnmove.x*scnmove.z*proj.x()
                + scnmove.y*scnmove.y*proj.y()*proj.y()
                - 2*scnmove.y*scnmove.z*proj.y()
                + scnmove.z*scnmove.z
                - proj.x()*proj.x()*Radius*Radius
                - proj.y()*proj.y()*Radius*Radius
                - Radius*Radius;
        B = - 2*scnmove.x*scnmove.x*scnmove.x*proj.x()
                - 2*scnmove.x*scnmove.x*scnmove.y*proj.y()
                - 2*scnmove.x*scnmove.x*scnmove.z*proj.x()*proj.x()
                + 2*scnmove.x*scnmove.x*scnmove.z
                - 2*scnmove.x*scnmove.y*scnmove.y*proj.x()
                - 4*scnmove.x*scnmove.y*scnmove.z*proj.x()*proj.y()
                + 2*scnmove.x*scnmove.z*scnmove.z*proj.x()
                + 2*scnmove.x*proj.x()*Radius*Radius
                - 2*scnmove.y*scnmove.y*scnmove.y*proj.y()
                - 2*scnmove.y*scnmove.y*scnmove.z*proj.y()*proj.y()
                + 2*scnmove.y*scnmove.y*scnmove.z
                + 2*scnmove.y*scnmove.z*scnmove.z*proj.y()
                + 2*scnmove.y*proj.y()*Radius*Radius
                + 2*scnmove.z*proj.x()*proj.x()*Radius*Radius
                + 2*scnmove.z*proj.y()*proj.y()*Radius*Radius;
        C = scnmove.x*scnmove.x*scnmove.x*scnmove.x
                + 2*scnmove.x*scnmove.x*scnmove.x*scnmove.z*proj.x()
                + 2*scnmove.x*scnmove.x*scnmove.y*scnmove.y
                + 2*scnmove.x*scnmove.x*scnmove.y*scnmove.z*proj.y()
                + scnmove.x*scnmove.x*scnmove.z*scnmove.z*proj.x()*proj.x()
                - scnmove.x*scnmove.x*Radius*Radius
                + 2*scnmove.x*scnmove.y*scnmove.y*scnmove.z*proj.x()
                + 2*scnmove.x*scnmove.y*scnmove.z*scnmove.z*proj.x()*proj.y()
                - 2*scnmove.x*scnmove.z*proj.x()*Radius*Radius
                + scnmove.y*scnmove.y*scnmove.y*scnmove.y
                + 2*scnmove.y*scnmove.y*scnmove.y*scnmove.z*proj.y()
                + scnmove.y*scnmove.y*scnmove.z*scnmove.z*proj.y()*proj.y()
                - scnmove.y*scnmove.y*Radius*Radius
                - 2*scnmove.y*scnmove.z*proj.y()*Radius*Radius
                - scnmove.z*scnmove.z*proj.x()*proj.x()*Radius*Radius
                - scnmove.z*scnmove.z*proj.y()*proj.y()*Radius*Radius;
        D  = B*B - 4*A*C;

        vec.z = (-B + sqrt(D))/(2*A);
        vec.x = proj.x() * (scnmove.z - vec.z) + scnmove.x;
        vec.y = proj.y() * (scnmove.z - vec.z) + scnmove.y;

    }
}

void M3D_CAMERA::moveScene(QMouseEvent *event)
{
    Vect3d  vec;
    QPointF LocalPt = MouseProjection(event->pos());

    // Векторное представление точки начала движения
    ProjectionToVect3d(MousePt, vec, LG_RADIUS);

    scnmove.x  += (LocalPt.x() - MousePt.x()) * (vec.z - scnmove.z);
    scnmove.y  += (LocalPt.y() - MousePt.y()) * (vec.z - scnmove.z);

    // Проверка что камера расположена на расстоянии LG_RADIUS+LG_NEAR_DIST от центра сферы
    GLdouble Z2 = ((LG_RADIUS+LG_NEAR_DIST)*(LG_RADIUS+LG_NEAR_DIST)) - scnmove.x*scnmove.x - scnmove.y*scnmove.y,
             Z  = Z2 > 0? sqrt(Z2) : 0.0;

    scnmove.z = qMax(Z, scnmove.z);

    Matrix4dTranslateFromVect3d(&matrix, &scnmove);
    MousePt = LocalPt;
}

void M3D_CAMERA::rotateLocal(QMouseEvent *event)
{
    Quat4d   q;
    Matrix3d m;
    Vect3d   v0, v1;
    QPointF  LocalPt  = MouseProjection(event->pos());

    // Векторное представление точки начала вращения
    ProjectionToVect3d(MousePt, v0, LG_RADIUS);

    // Векторное представление точки окончания вращения (относительно центра Земли)
    ProjectionToVect3d(LocalPt, v1, LG_RADIUS);

    rotateToQuat4d(&v0, &v1, &q);
    Quat4dToMatrix3d(&q, &m);
    MulMatrix3d(&m, &locrotate);

    Matrix4dRotateFromMatrix3d(&matrix, &locrotate);
    MousePt  = LocalPt;
}

void M3D_CAMERA::mouseMoveEvent(QMouseEvent *event)
{
    if(m_rotateLocal)
    {
        rotateLocal(event);
        updateGL();
    }
    else if(m_moveScene)
    {
        moveScene(event);
        updateGL();
    }
}

void M3D_CAMERA::wheelEvent(QWheelEvent *event)
{
    Vect3d   vec;
    QPointF  proj = MouseProjection(event->pos());
    GLdouble distance = scnmove.x*scnmove.x+scnmove.y*scnmove.y+scnmove.z*scnmove.z-(LG_RADIUS+LG_NEAR_DIST)*(LG_RADIUS+LG_NEAR_DIST);
    GLdouble step = (event->delta() > 0? 1.0 : -1.0) *
             (LG_CAM_SPEED*sqrt(distance > 0.0 ? distance : 0.0));

    ProjectionToVect3d(proj, vec, LG_RADIUS+LG_NEAR_DIST);
    step = qBound(scnmove.z - LG_FAR_DIST+LG_RADIUS+LG_NEAR_DIST, step, scnmove.z - vec.z);
    scnmove.x += step * proj.x();
    scnmove.y += step * proj.y();
    scnmove.z -= step;

    Matrix4dTranslateFromVect3d(&matrix, &scnmove);
    updateGL();
}

void M3D_CAMERA::mouseReleaseEvent(QMouseEvent *event)
{
    (void)event;
    m_rotateLocal = 0;
    m_moveScene = 0;
    setCursor(Qt::OpenHandCursor);
}

void M3D_CAMERA::resizeGL(int w, int h)
{
    GLdouble ratio = (GLdouble)w / h,
             fH = tan(LG_FOVY * M_DEG_TO_RAD) * LG_NEAR_DIST,
             fW = fH * ratio;

    glViewport(0, 0, w, h);

    // Генерация матрицы проекций
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    glFrustum(-fW, fW, -fH, fH, LG_NEAR_DIST, LG_FAR_DIST);
}

void M3D_CAMERA::setModelViewMatrix()
{
    // Генерация модельно-видовой матрицы
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();

    glMultMatrixd((GLdouble*)&matrix);
}




CMap3d::CMap3d(QWidget *parent) : M3D_CAMERA(parent),
    earthText(0),glCompressedTexImage2D(NULL)
{
    setWindowTitle(tr("3D МОДЕЛЬ ЗЕМЛИ"));
    setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
    setMouseTracking(true);
    setFocusPolicy(Qt::StrongFocus);

    Message.clear();
    setCursor(Qt::OpenHandCursor);

    Vertex.reserve(3*DECL_PATH_ELEM);
    coordVertexSphere.reserve(6*LG_SLICE_CNT*LG_SLICE_CNT);
    normalVertexSphere.reserve(6*LG_SLICE_CNT*LG_SLICE_CNT);
    textureVertexSphere.reserve(4*LG_SLICE_CNT*LG_SLICE_CNT);

    // Инициализация пользователяского интерфейса
    mapLayout = new QGridLayout(this);
    mapLayout->setContentsMargins(5, 5, 5, 5);

    visibleMapper = new QSignalMapper(this);
    connect(visibleMapper, SIGNAL(mapped(int)), SLOT(mappingButtons(int)));

    menuButton = new QToolButton(this);
    menuButton->setIcon(QIcon(":/Resources/img_ok.png"));
    menuButton->setIconSize(QSize(16, 16));
    menuButton->setCheckable(true);
    mapLayout->addWidget(menuButton, 0, 1, Qt::AlignRight | Qt::AlignTop);

    connect(menuButton, SIGNAL(toggled(bool)), SLOT(showButtons(bool)));

    InsertButtons(show_light,true,tr("Освещение"),tr("освещение"));
    InsertButtons(default_pos,false,tr("Начало"),tr("начальное состояние"));

    visibleObjectsButton.value(default_pos)->setCheckable(false);
    connect(visibleObjectsButton.value(default_pos), SIGNAL(clicked()),this, SLOT(initPosition()));

    mapLayout->setColumnStretch(0, 1);
    mapLayout->setRowStretch(mapLayout->rowCount(), 1);
}

CMap3d::~CMap3d()
{
    makeCurrent();
    Vertex.clear();
    coordVertexSphere.clear();
    normalVertexSphere.clear();
    textureVertexSphere.clear();
    isVisibleObject.clear();
    visibleObjectsButton.clear();
    glDeleteTextures(1,&earthText);
}

void CMap3d::MESSAGE(QString text)
{
    Message += text + QString("\n");
}

void CMap3d::InsertButtons(const ObjectsVisibleOnMap key, bool value, QString name, QString text)
{
    // Создание и инициализация начального значения
    isVisibleObject.insert(key, value);

    // Создание кнопки
    QPushButton* btn = new QPushButton(name, this);

    btn->hide();
    btn->setCheckable(true);
    btn->setChecked(isVisibleObject.value(key));
    mapLayout->addWidget(btn, mapLayout->rowCount(), 1);

    btn->setToolTip(tr("Отобразить на карте %1").arg(text));
    btn->setStatusTip(tr("Отобразить на карте %1").arg(text));
    visibleObjectsButton.insert(key, btn);

    // Связывание сигнала от QPushButton с QSignalMapper
    connect(btn, SIGNAL(clicked()), visibleMapper, SLOT(map()));
    visibleMapper->setMapping(btn, key);
}

void CMap3d::mappingButtons(int value)
{
    ObjectsVisibleOnMap key = static_cast<ObjectsVisibleOnMap>(value);
    isVisibleObject[key] = !isVisibleObject.value(key);
    visibleObjectsButton.value(key)->setChecked(isVisibleObject.value(key));
    repaint();
}

void CMap3d::showButtons(bool state)
{
        QMapIterator<ObjectsVisibleOnMap, QPushButton*> i(visibleObjectsButton);
        while(i.hasNext()) {
            i.next();
            if(state == true)
                i.value()->show();
            else
                i.value()->hide();
        }
}

void CMap3d::initPosition()
{
    initCamera(0.0, 0.0, SHIRMPSK, DOLGMPSK, 3.0);
    updateGL();
}


void CMap3d::createSphere(GLdouble xc, GLdouble yc, GLdouble zc, GLdouble r)
{
    int
            i,j;
    GLdouble
            theta, phi,
            stepAngle   = M_2PI / LG_SLICE_CNT,
            stepTexture = 1.0 / LG_SLICE_CNT;

    coordVertexSphere.clear();
    normalVertexSphere.clear();
    textureVertexSphere.clear();

    for(j = 0; j < LG_SLICE_CNT / 2; j++)
    {
        // -pi/2 .. +pi/2
        theta = j * stepAngle - M_PI/2.0;

        for(i = 0; i < LG_SLICE_CNT; i++)
        {
            // -pi .. +pi
            phi = -M_PI + i*stepAngle;
            //X - вправо , Y - вверх , Z- на нас

            // Координаты привязки
            coordVertexSphere.push_back( xc + r * cos(theta+stepAngle) * sin(phi));
            coordVertexSphere.push_back( yc + r * sin(theta+stepAngle));
            coordVertexSphere.push_back( zc + r * cos(theta+stepAngle) * cos(phi));
            // Нормаль
            normalVertexSphere.push_back( cos(theta+stepAngle) * sin(phi));
            normalVertexSphere.push_back( sin(theta+stepAngle));
            normalVertexSphere.push_back( cos(theta+stepAngle) * cos(phi));
            // Текстурные кординаты
            textureVertexSphere.push_back( i * stepTexture);
            textureVertexSphere.push_back( 2.0 * (j+1)* stepTexture);

            // Координаты привязки
            coordVertexSphere.push_back( xc + r * cos(theta) * sin(phi));
            coordVertexSphere.push_back( yc + r * sin(theta));
            coordVertexSphere.push_back( zc + r * cos(theta) * cos(phi));
            // Нормаль
            normalVertexSphere.push_back( cos(theta) * sin(phi));
            normalVertexSphere.push_back( sin(theta));
            normalVertexSphere.push_back( cos(theta) * cos(phi));
            // Текстурные кординаты
            textureVertexSphere.push_back( i * stepTexture);
            textureVertexSphere.push_back( 2.0 *j * stepTexture);

            // Координаты привязки
            coordVertexSphere.push_back( xc + r * cos(theta) * sin(phi+stepAngle));
            coordVertexSphere.push_back( yc + r * sin(theta));
            coordVertexSphere.push_back( zc + r * cos(theta) * cos(phi+stepAngle));
            // Нормаль
            normalVertexSphere.push_back( cos(theta) * sin(phi+stepAngle));
            normalVertexSphere.push_back( sin(theta));
            normalVertexSphere.push_back( cos(theta) * cos(phi+stepAngle));
            // Текстурные кординаты
            textureVertexSphere.push_back( (i+1) * stepTexture);
            textureVertexSphere.push_back( 2.0 *j * stepTexture);

            // Координаты привязки
            coordVertexSphere.push_back( xc + r * cos(theta+stepAngle) * sin(phi+stepAngle));
            coordVertexSphere.push_back( yc + r * sin(theta+stepAngle));
            coordVertexSphere.push_back( zc + r * cos(theta+stepAngle) * cos(phi+stepAngle));
            // Нормаль
            normalVertexSphere.push_back( cos(theta+stepAngle) * sin(phi+stepAngle));
            normalVertexSphere.push_back( sin(theta+stepAngle));
            normalVertexSphere.push_back( cos(theta+stepAngle) * cos(phi+stepAngle));
            // Текстурные кординаты
            textureVertexSphere.push_back( (i+1) * stepTexture);
            textureVertexSphere.push_back( 2.0 * (j+1)* stepTexture);
        }
    }
}


void CMap3d::loadTexture()
{
    // Макрос для чтения беззнакового целого значения LITTLE ENDIAN (платформонезависимый)
    #define LE_UINT32(a)((unsigned int)((unsigned char)(a)[3] << 24) | ((unsigned char)(a)[2] << 16) | ((unsigned char)(a)[1] << 8) | ((unsigned char)(a)[0]))

    #define HEADER_PNG  0x474E5089UL
    #define HEADER_JPG  0xE0FFD8FFUL
    #define HEADER_GIF  0x38464947UL
    // Чтение сжатой тестуры в формате DDS
    // Источник: http://opengl-tutorial.blogspot.com/p/5.html
    //           https://pastebin.com/XanZ4pP7
    //           https://coderoad.ru/33459066/%D0%97%D0%B0%D0%B3%D1%80%D1%83%D0%B7%D0%BA%D0%B0-%D1%82%D0%B5%D0%BA%D1%81%D1%82%D1%83%D1%80-DDS

    #define HEADER_DDS  0x20534444UL // Equivalent to "DDS " in ASCII
    #define FOURCC_DXT1 0x31545844UL // Equivalent to "DXT1" in ASCII
    #define FOURCC_DXT3 0x33545844UL // Equivalent to "DXT3" in ASCII
    #define FOURCC_DXT5 0x35545844UL // Equivalent to "DXT5" in ASCII

    // Константы форматов сжатых текстур для glCompressedTexImage2D
    // Источник: https://www.khronos.org/registry/OpenGL/extensions/EXT/EXT_texture_compression_s3tc.txt
    #define GL_COMPRESSED_RGB_S3TC_DXT1_EXT    0x83F0U
    #define GL_COMPRESSED_RGBA_S3TC_DXT1_EXT   0x83F1U
    #define GL_COMPRESSED_RGBA_S3TC_DXT3_EXT   0x83F2U
    #define GL_COMPRESSED_RGBA_S3TC_DXT5_EXT   0x83F3U

    #define GL_TEXTURE_BASE_LEVEL              0x813CU
    #define GL_TEXTURE_MAX_LEVEL               0x813DU

    QFile file(":/Resources/earth.dds");

    if (file.open(QFile::ReadOnly))
    {
        GLint maxTexSize;
        QByteArray array = file.readAll();

        glGetIntegerv(GL_MAX_TEXTURE_SIZE, &maxTexSize);

        // Выделение памяти и инициализация параметров текстуры
        glGenTextures(1, &earthText);
        glBindTexture(GL_TEXTURE_2D, earthText);

        // При фильтрации игнорируются тексели, выходящие за границу текстуры для s координаты
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
        // При фильтрации игнорируются тексели, выходящие за границу текстуры для t координаты
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
        // Цвет текселя полностью замещает цвет фрагмента фигуры
        glTexEnvi(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_MODULATE);

        switch (LE_UINT32(array.data()))
        {
        case HEADER_PNG:
        case HEADER_JPG:
        case HEADER_GIF:
            {
                QImage  image = QImage::fromData(array);
                // Проверка размеров текстуры
                if (image.width() <= maxTexSize && image.height() <= maxTexSize)
                {
                    // Конвертация в формат OpenGL
                    image = QGLWidget::convertToGLFormat(image);
                    // Дополнительные параметры текстурного объекта
                    // Задание линейную фильтрацию вблизи:
                    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
                    // Задание линейной фильтрации вдали:
                    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
                    // Загрузка текстуры в память видеокарты
                    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, image.width(), image.height(), 0, GL_RGBA, GL_UNSIGNED_BYTE, image.bits());
                }
                else
                {
                    MESSAGE(tr("Размер текстуры %1x%2 превышает поддерживаемый размер текстуры %3").arg(image.width()).arg(image.height()).arg(maxTexSize));
                }
                }
            break;
        case HEADER_DDS:
        {
            if (array.size() > 128)
            {
                unsigned int height        = LE_UINT32(array.data()+12);
                unsigned int width         = LE_UINT32(array.data()+16);
                unsigned int mipMapCount   = LE_UINT32(array.data()+28);
                unsigned int fourCC        = LE_UINT32(array.data()+84);

                GLenum format = 0;
                unsigned int blockSize = 0;

                switch(fourCC)
                {
                   case FOURCC_DXT1:
                     format = GL_COMPRESSED_RGB_S3TC_DXT1_EXT;
                     blockSize = 8;
                     break;
                   case FOURCC_DXT3:
                     format = GL_COMPRESSED_RGBA_S3TC_DXT3_EXT;
                     blockSize = 16;
                     break;
                   case FOURCC_DXT5:
                     format = GL_COMPRESSED_RGBA_S3TC_DXT5_EXT;
                     blockSize = 16;
                     break;
                   default:
                     format = 0;
                     break;
                 }

                 GLint num = 0;
                 glGetIntegerv(GL_NUM_COMPRESSED_TEXTURE_FORMATS, &num);

                 QVector<GLint> supported(num);
                 glGetIntegerv(GL_COMPRESSED_TEXTURE_FORMATS, supported.data());

                 if (supported.contains(format))
                 {
                    int offset    = 128;

                    glPixelStorei(GL_UNPACK_ALIGNMENT,1);

                    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_BASE_LEVEL, 0);
                    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAX_LEVEL, mipMapCount-1);
                    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
                    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);

                    // Проверка размеров текстуры
                    if (width <= (unsigned int)maxTexSize && height <= (unsigned int)maxTexSize)
                    {
                        // Загрузка текстуры в память видеокарты
                        for (unsigned int level = 0; level < mipMapCount; ++level)
                        {
                           int size = ((width+3)/4)*((height+3)/4)*blockSize;

                           if (array.size() >= offset + size)
                           {
                               if (glCompressedTexImage2D != NULL)
                                   glCompressedTexImage2D(GL_TEXTURE_2D, level, format, width, height, 0, size, array.data() + offset);
                               offset += size;
                               width  /= 2;
                               height /= 2;
                           }
                        }
                    }
                    else
                    {
                        MESSAGE(tr("Размер текстуры %1x%2 превышает поддерживаемый размер текстуры %3").arg(width).arg(height).arg(maxTexSize));
                    }
                 }
                 else
                 {
                     MESSAGE(tr("Формат сжатой текстуры S3TC 0x%1 (0x%2) не поддерживается OpenGL").arg(format, 0, 16).arg(fourCC, 0, 16));
                 }
            }
        break;
        } // case HEADER_DDS:
        default:
            {
                MESSAGE(tr("Формат текстуры 0x%1 не поддерживается загрузчиком текстур").arg(LE_UINT32(array.data()), 0, 16));
                break;
            }
        }
        file.close();

    }
    else
    {
        MESSAGE(tr("Ошибка чтения текстуры из ресурсов"));
    }
}

void CMap3d::createEarth()
{
    loadTexture();
    createSphere(0.0, 0.0, 0.0, LG_RADIUS);
}


void CMap3d::initializeGL()
{
    #if defined(_MSC_VER)
        glCompressedTexImage2D = (GLCOMPRESSEDTEXIMAGE2D)wglGetProcAddress("glCompressedTexImage2D");
    #else
        glCompressedTexImage2D = (GLCOMPRESSEDTEXIMAGE2D)context()->getProcAddress(QLatin1String("glCompressedTexImage2D"));
    #endif

    glClearColor(0.0f, 0.0f, 0.0f, 0.5f);
    glShadeModel(GL_SMOOTH);
    glClearDepth(1.0f);
    glEnable(GL_DEPTH_TEST);
    glDepthFunc(GL_LEQUAL);
    glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST);

    glColor4f(1.0f, 1.0f, 1.0f, 0.5f);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE);
    glDisable(GL_BLEND);
    glEnable(GL_DEPTH_TEST);

    glCullFace(GL_BACK);
    glEnable(GL_CULL_FACE);

    glEnable(GL_LINE_SMOOTH);
    glHint(GL_LINE_SMOOTH_HINT,GL_NICEST);
    glEnable(GL_POINT_SMOOTH);
    glHint(GL_POINT_SMOOTH_HINT,GL_NICEST);

    createEarth();
    initCamera(0.0, 0.0, SHIRMPSK, DOLGMPSK, 3.0);

}




void CMap3d::paintGL()
{
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    setModelViewMatrix();

    //! Вывод сообщения об ошибке
    if (!Message.isEmpty())
    {
        qglColor(Qt::white);
        renderText(15, 15, Message);
    }
    else

    //! Штатное отображение карты
    {
        qglColor(Qt::white);


        // Отображение освещения
        if(isVisibleObject[show_light])
            drawSunLight();

        // Отображение сферы с наложенной на неё текстурой.
        drawEarth();

        glDisable(GL_LIGHTING);
        glDisable(GL_LIGHT0);

        glPopMatrix();
    }

}


void CMap3d::drawEarth()
{
    // Отрисовка сферы
    glEnable(GL_TEXTURE_2D);
    glColor4d(1.0, 1.0, 1.0, 0.0);

    glBindTexture(GL_TEXTURE_2D, earthText);

    glEnableClientState(GL_VERTEX_ARRAY);
    glEnableClientState(GL_NORMAL_ARRAY);
    glEnableClientState(GL_TEXTURE_COORD_ARRAY);

    glVertexPointer(3,GL_DOUBLE,0,coordVertexSphere.data());
    glNormalPointer(GL_DOUBLE,0,normalVertexSphere.data());
    glTexCoordPointer(2,GL_DOUBLE,0,textureVertexSphere.data());
    glDrawArrays(GL_QUADS,0,2*LG_SLICE_CNT*LG_SLICE_CNT);

    glDisableClientState(GL_VERTEX_ARRAY);
    glDisableClientState(GL_NORMAL_ARRAY);
    glDisableClientState(GL_TEXTURE_COORD_ARRAY);

    glDisable(GL_TEXTURE_2D);
}


void CMap3d::drawSunLight()
{

	// РЕШЕНИЕ ЗАДАЧИ ОБ ОПРЕДЕЛЕНИИ ПОДСОЛНЕЧНОЙ ТОЧКИ
	// Монтенбрюк О., Пфлегер Т. Астрономия на персональном компьютере. Питер, 2002. 320 с.
	// Jean Meeus, Astronomical Algorithms, Second Edition. Publisher: Willmann-Bell 1999
	// http://www.astroclub.kiev.ua/forum/index.php?topic=5525.0

	// Определение модифицированной Юлианской Даты
    double mjd = std::time(NULL)/86400.0 + 40587.0,

	// Вычисление гринвичского среднего звездного времени GMST, рад
	mjd0  = (int)mjd,
	UT1   = 86400.0*(mjd-mjd0),
	T0    = (mjd0-51544.5)/36525,
	T     = (mjd - 51544.5)/36525,
	GMST  = (24110.54841 + 8640184.812866*T0 + (0.093104-6.2e-6*T0)*T0*T0 + 1.002737909350795*UT1) * 2.0*M_PI/86400;

	// Положение Солнца в эклиптической системе координат:
	// Средняя аномалия Солнца, рад
    double M0  = fmod(357.52910 + (35999.05030 - (0.0001559 + 0.00000048*T)*T)*T, 360.0) * M_DEG_TO_RAD,
	// Геометрическая средняя долгота Солнца, приведённая к среднему текущему равноденствию, рад
	L0  = 280.46645 + (36000.76983 + 0.0003032*T)*T,
	// Истинная долгота Солнца (Геометрическая средняя долгота Солнца + Положение центра диска Солнца), рад
	L   = fmod(L0 + (1.914600 - (0.004817*T + 0.00014*T)*T)* sin(M0) +
			(0.019993 - 0.000101*T)* sin(2*M0) + 0.000290* sin(3*M0), 360.0) * M_DEG_TO_RAD,
	// Угол наклона оси вращения Земли, рад
	EPS = ( 23.43929111-(46.8150+(0.00059-0.001813*T)*T)*T/3600.0) * M_DEG_TO_RAD,

	// Экваториальные геоцентрические координаты Солнца:
	// Прямое восхождение Солнца (Долгота подсолнечной точки без учета суточного вращения)
	Ra  = atan2(sin(L) * cos(EPS), cos(L)),
	// Склонение Солнца (Широта подсолнечной точки)
	Dec = asin(sin(L) * sin(EPS));

	// Положение Солнца (направление)
	float SunPos[] = { cos(Dec)*sin(Ra-GMST), sin(Dec), cos(Dec)*cos(Ra-GMST), 0.0f };
	// Фоновое освещение
	float fAmbientShadow[] = { 0.5f, 0.5f, 0.5f, 1.0f };

	// Начальная позиция источника света и интенсивность тёмной части.
	glLightfv(GL_LIGHT0, GL_POSITION, SunPos);
	glLightModelfv(GL_LIGHT_MODEL_AMBIENT, fAmbientShadow);

	// Включаем источник света.
	glEnable(GL_LIGHTING);
	glEnable(GL_LIGHT0);


}
