/****************************************************************************
** Meta object code from reading C++ file 'packetstatisticsdialog.h'
**
** Created: Thu Jun 27 00:13:11 2013
**      by: The Qt Meta Object Compiler version 63 (Qt 4.8.1)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "animator/packetstatisticsdialog.h"
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'packetstatisticsdialog.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 63
#error "This file was generated using the moc from 4.8.1. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
static const uint qt_meta_data_netanim__Packetstatisticsdialog[] = {

 // content:
       6,       // revision
       0,       // classname
       0,    0, // classinfo
       3,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: signature, parameters, type, tag, flags
      33,   32,   32,   32, 0x09,
      57,   32,   32,   32, 0x09,
      93,   32,   32,   32, 0x09,

       0        // eod
};

static const char qt_meta_stringdata_netanim__Packetstatisticsdialog[] = {
    "netanim::Packetstatisticsdialog\0\0"
    "applyPacketFilterSlot()\0"
    "packetFilterProtocolSelectAllSlot()\0"
    "packetFilterProtocolDeSelectAllSlot()\0"
};

void netanim::Packetstatisticsdialog::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        Q_ASSERT(staticMetaObject.cast(_o));
        Packetstatisticsdialog *_t = static_cast<Packetstatisticsdialog *>(_o);
        switch (_id) {
        case 0: _t->applyPacketFilterSlot(); break;
        case 1: _t->packetFilterProtocolSelectAllSlot(); break;
        case 2: _t->packetFilterProtocolDeSelectAllSlot(); break;
        default: ;
        }
    }
    Q_UNUSED(_a);
}

const QMetaObjectExtraData netanim::Packetstatisticsdialog::staticMetaObjectExtraData = {
    0,  qt_static_metacall 
};

const QMetaObject netanim::Packetstatisticsdialog::staticMetaObject = {
    { &QDialog::staticMetaObject, qt_meta_stringdata_netanim__Packetstatisticsdialog,
      qt_meta_data_netanim__Packetstatisticsdialog, &staticMetaObjectExtraData }
};

#ifdef Q_NO_DATA_RELOCATION
const QMetaObject &netanim::Packetstatisticsdialog::getStaticMetaObject() { return staticMetaObject; }
#endif //Q_NO_DATA_RELOCATION

const QMetaObject *netanim::Packetstatisticsdialog::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->metaObject : &staticMetaObject;
}

void *netanim::Packetstatisticsdialog::qt_metacast(const char *_clname)
{
    if (!_clname) return 0;
    if (!strcmp(_clname, qt_meta_stringdata_netanim__Packetstatisticsdialog))
        return static_cast<void*>(const_cast< Packetstatisticsdialog*>(this));
    return QDialog::qt_metacast(_clname);
}

int netanim::Packetstatisticsdialog::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QDialog::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 3)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 3;
    }
    return _id;
}
QT_END_MOC_NAMESPACE
