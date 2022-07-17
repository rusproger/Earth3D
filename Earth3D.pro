QT       += core gui opengl

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

#CONFIG += c++11

unix {
        DEFINES  += _POSIX_API_
}

win32 {
        DEFINES += _WIN_API_

        win32-msvc* {
        QMAKE_EXTRA_TARGETS +=before_build makefilehook

        makefilehook.target = $(MAKEFILE)
        makefilehook.depends = .beforebuild

        before_build.target = .beforebuild
        before_build.depends = FORCE
        before_build.commands = chcp 1251
        }
}

SOURCES += \
    main.cpp \
    map3d.cpp

HEADERS += \
    map3d.h

FORMS +=


RESOURCES +=     resources.qrc

# Default rules for deployment.
qnx: target.path = /tmp/$${TARGET}/bin
else: unix:!android: target.path = /opt/$${TARGET}/bin
!isEmpty(target.path): INSTALLS += target
