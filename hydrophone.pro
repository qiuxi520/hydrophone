#-------------------------------------------------
#
# Project created by QtCreator 2017-04-26
#
#-------------------------------------------------
QT       += core gui printsupport

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets network

RC_ICONS = Resources\hat.ico
TARGET = AI_StreamingAI
TEMPLATE = app

SOURCES +=  configuredialog.cpp\ \
            main.cpp \
            qcustomplot.cpp \
            streamingbufferedai.cpp



HEADERS +=  configuredialog.h\
            bdaqctrl.h \
            qcustomplot.h \
            streamingbufferedai.h

FORMS   +=  configuredialog.ui \
            streamingbufferedai.ui

RESOURCES += streamingbufferedai.qrc

CONFIG += debug_and_release

# 添加 winmm 库
LIBS += -lwinmm

CONFIG(debug, debug|release){
        DESTDIR += $$PWD/../bin/debug
        OBJECTS_DIR = $$PWD/debug
        UI_DIR      = $$PWD/debug/ui
        MOC_DIR     = $$PWD/debug/moc
        RCC_DIR     = $$PWD/debug/rcc

} else {
        DESTDIR += $$PWD/../bin/release
        OBJECTS_DIR = $$PWD/release
        UI_DIR      = $$PWD/release/ui
        MOC_DIR     = $$PWD/release/moc
        RCC_DIR     = $$PWD/release/rcc
}
