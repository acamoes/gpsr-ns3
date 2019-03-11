/****************************************************************************
** Meta object code from reading C++ file 'animatormode.h'
**
** Created: Thu Jun 27 00:13:10 2013
**      by: The Qt Meta Object Compiler version 63 (Qt 4.8.1)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "animator/animatormode.h"
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'animatormode.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 63
#error "This file was generated using the moc from 4.8.1. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
static const uint qt_meta_data_netanim__AnimatorMode[] = {

 // content:
       6,       // revision
       0,       // classname
       0,    0, // classinfo
      25,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: signature, parameters, type, tag, flags
      23,   22,   22,   22, 0x08,
      34,   22,   22,   22, 0x08,
      59,   22,   22,   22, 0x08,
      77,   22,   22,   22, 0x08,
      96,   22,   22,   22, 0x08,
     112,   22,   22,   22, 0x08,
     129,   22,   22,   22, 0x08,
     145,   22,   22,   22, 0x08,
     177,  171,   22,   22, 0x08,
     214,  171,   22,   22, 0x08,
     238,   22,   22,   22, 0x08,
     262,  171,   22,   22, 0x08,
     287,  171,   22,   22, 0x08,
     315,   22,   22,   22, 0x08,
     341,   22,   22,   22, 0x08,
     361,   22,   22,   22, 0x08,
     378,   22,   22,   22, 0x08,
     393,   22,   22,   22, 0x08,
     410,   22,   22,   22, 0x08,
     436,   22,   22,   22, 0x08,
     458,   22,   22,   22, 0x08,
     486,   22,   22,   22, 0x08,
     499,   22,   22,   22, 0x08,
     513,   22,   22,   22, 0x08,
     535,   22,   22,   22, 0x08,

       0        // eod
};

static const char qt_meta_stringdata_netanim__AnimatorMode[] = {
    "netanim::AnimatorMode\0\0testSlot()\0"
    "clickTraceFileOpenSlot()\0clickZoomInSlot()\0"
    "clickZoomOutSlot()\0clickSaveSlot()\0"
    "clickResetSlot()\0clickPlaySlot()\0"
    "clickAddCustomImageSlot()\0value\0"
    "updatePacketPersistenceSlot(QString)\0"
    "updateTimelineSlot(int)\0updateRateTimeoutSlot()\0"
    "updateGridLinesSlot(int)\0"
    "updateNodeSizeSlot(QString)\0"
    "updateUpdateRateSlot(int)\0showGridLinesSlot()\0"
    "showNodeIdSlot()\0showMetaSlot()\0"
    "showPacketSlot()\0showWirelessCirclesSlot()\0"
    "showPacketStatsSlot()\0showNodePositionStatsSlot()\0"
    "showIpSlot()\0showMacSlot()\0"
    "setUnicastMatchSlot()\0showRoutePathSlot()\0"
};

void netanim::AnimatorMode::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        Q_ASSERT(staticMetaObject.cast(_o));
        AnimatorMode *_t = static_cast<AnimatorMode *>(_o);
        switch (_id) {
        case 0: _t->testSlot(); break;
        case 1: _t->clickTraceFileOpenSlot(); break;
        case 2: _t->clickZoomInSlot(); break;
        case 3: _t->clickZoomOutSlot(); break;
        case 4: _t->clickSaveSlot(); break;
        case 5: _t->clickResetSlot(); break;
        case 6: _t->clickPlaySlot(); break;
        case 7: _t->clickAddCustomImageSlot(); break;
        case 8: _t->updatePacketPersistenceSlot((*reinterpret_cast< QString(*)>(_a[1]))); break;
        case 9: _t->updateTimelineSlot((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 10: _t->updateRateTimeoutSlot(); break;
        case 11: _t->updateGridLinesSlot((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 12: _t->updateNodeSizeSlot((*reinterpret_cast< QString(*)>(_a[1]))); break;
        case 13: _t->updateUpdateRateSlot((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 14: _t->showGridLinesSlot(); break;
        case 15: _t->showNodeIdSlot(); break;
        case 16: _t->showMetaSlot(); break;
        case 17: _t->showPacketSlot(); break;
        case 18: _t->showWirelessCirclesSlot(); break;
        case 19: _t->showPacketStatsSlot(); break;
        case 20: _t->showNodePositionStatsSlot(); break;
        case 21: _t->showIpSlot(); break;
        case 22: _t->showMacSlot(); break;
        case 23: _t->setUnicastMatchSlot(); break;
        case 24: _t->showRoutePathSlot(); break;
        default: ;
        }
    }
}

const QMetaObjectExtraData netanim::AnimatorMode::staticMetaObjectExtraData = {
    0,  qt_static_metacall 
};

const QMetaObject netanim::AnimatorMode::staticMetaObject = {
    { &Mode::staticMetaObject, qt_meta_stringdata_netanim__AnimatorMode,
      qt_meta_data_netanim__AnimatorMode, &staticMetaObjectExtraData }
};

#ifdef Q_NO_DATA_RELOCATION
const QMetaObject &netanim::AnimatorMode::getStaticMetaObject() { return staticMetaObject; }
#endif //Q_NO_DATA_RELOCATION

const QMetaObject *netanim::AnimatorMode::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->metaObject : &staticMetaObject;
}

void *netanim::AnimatorMode::qt_metacast(const char *_clname)
{
    if (!_clname) return 0;
    if (!strcmp(_clname, qt_meta_stringdata_netanim__AnimatorMode))
        return static_cast<void*>(const_cast< AnimatorMode*>(this));
    return Mode::qt_metacast(_clname);
}

int netanim::AnimatorMode::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = Mode::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 25)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 25;
    }
    return _id;
}
QT_END_MOC_NAMESPACE
