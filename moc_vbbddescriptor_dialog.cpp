/****************************************************************************
** Meta object code from reading C++ file 'vbbddescriptor_dialog.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.6.0)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "vbbddescriptor_dialog.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'vbbddescriptor_dialog.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.6.0. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
struct qt_meta_stringdata_AutoComputeAllDialog_t {
    QByteArrayData data[7];
    char stringdata0[104];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_AutoComputeAllDialog_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_AutoComputeAllDialog_t qt_meta_stringdata_AutoComputeAllDialog = {
    {
QT_MOC_LITERAL(0, 0, 20), // "AutoComputeAllDialog"
QT_MOC_LITERAL(1, 21, 14), // "setValue_index"
QT_MOC_LITERAL(2, 36, 0), // ""
QT_MOC_LITERAL(3, 37, 15), // "SetValue_Radius"
QT_MOC_LITERAL(4, 53, 17), // "SetValue_GridSize"
QT_MOC_LITERAL(5, 71, 17), // "SetValue_Sampling"
QT_MOC_LITERAL(6, 89, 14) // "SetValue_ratio"

    },
    "AutoComputeAllDialog\0setValue_index\0"
    "\0SetValue_Radius\0SetValue_GridSize\0"
    "SetValue_Sampling\0SetValue_ratio"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_AutoComputeAllDialog[] = {

 // content:
       7,       // revision
       0,       // classname
       0,    0, // classinfo
       5,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: name, argc, parameters, tag, flags
       1,    1,   39,    2, 0x09 /* Protected */,
       3,    1,   42,    2, 0x09 /* Protected */,
       4,    1,   45,    2, 0x09 /* Protected */,
       5,    1,   48,    2, 0x09 /* Protected */,
       6,    1,   51,    2, 0x09 /* Protected */,

 // slots: parameters
    QMetaType::Void, QMetaType::Int,    2,
    QMetaType::Void, QMetaType::Int,    2,
    QMetaType::Void, QMetaType::Int,    2,
    QMetaType::Void, QMetaType::Int,    2,
    QMetaType::Void, QMetaType::Double,    2,

       0        // eod
};

void AutoComputeAllDialog::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        AutoComputeAllDialog *_t = static_cast<AutoComputeAllDialog *>(_o);
        Q_UNUSED(_t)
        switch (_id) {
        case 0: _t->setValue_index((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 1: _t->SetValue_Radius((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 2: _t->SetValue_GridSize((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 3: _t->SetValue_Sampling((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 4: _t->SetValue_ratio((*reinterpret_cast< double(*)>(_a[1]))); break;
        default: ;
        }
    }
}

const QMetaObject AutoComputeAllDialog::staticMetaObject = {
    { &QDialog::staticMetaObject, qt_meta_stringdata_AutoComputeAllDialog.data,
      qt_meta_data_AutoComputeAllDialog,  qt_static_metacall, Q_NULLPTR, Q_NULLPTR}
};


const QMetaObject *AutoComputeAllDialog::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *AutoComputeAllDialog::qt_metacast(const char *_clname)
{
    if (!_clname) return Q_NULLPTR;
    if (!strcmp(_clname, qt_meta_stringdata_AutoComputeAllDialog.stringdata0))
        return static_cast<void*>(const_cast< AutoComputeAllDialog*>(this));
    if (!strcmp(_clname, "Ui::VBBDDescriptorDialog"))
        return static_cast< Ui::VBBDDescriptorDialog*>(const_cast< AutoComputeAllDialog*>(this));
    return QDialog::qt_metacast(_clname);
}

int AutoComputeAllDialog::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QDialog::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 5)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 5;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 5)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 5;
    }
    return _id;
}
QT_END_MOC_NAMESPACE
