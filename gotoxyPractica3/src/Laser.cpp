//
// Copyright (c) ZeroC, Inc. All rights reserved.
//
//
// Ice version 3.7.3
//
// <auto-generated>
//
// Generated from file `Laser.ice'
//
// Warning: do not edit this file.
//
// </auto-generated>
//

#include <Laser.h>
#include <IceUtil/PushDisableWarnings.h>
#include <Ice/LocalException.h>
#include <Ice/ValueFactory.h>
#include <Ice/OutgoingAsync.h>
#include <Ice/InputStream.h>
#include <Ice/OutputStream.h>
#include <Ice/LocalException.h>
#include <IceUtil/PopDisableWarnings.h>

#if defined(_MSC_VER)
#   pragma warning(disable:4458) // declaration of ... hides class member
#elif defined(__clang__)
#   pragma clang diagnostic ignored "-Wshadow"
#elif defined(__GNUC__)
#   pragma GCC diagnostic ignored "-Wshadow"
#endif

#ifndef ICE_IGNORE_VERSION
#   if ICE_INT_VERSION / 100 != 307
#       error Ice version mismatch!
#   endif
#   if ICE_INT_VERSION % 100 >= 50
#       error Beta header file detected
#   endif
#   if ICE_INT_VERSION % 100 < 3
#       error Ice patch level mismatch!
#   endif
#endif

#ifdef ICE_CPP11_MAPPING // C++11 mapping

namespace
{

const ::std::string iceC_RoboCompLaser_Laser_ids[2] =
{
    "::Ice::Object",
    "::RoboCompLaser::Laser"
};
const ::std::string iceC_RoboCompLaser_Laser_ops[] =
{
    "getLaserAndBStateData",
    "getLaserConfData",
    "getLaserData",
    "ice_id",
    "ice_ids",
    "ice_isA",
    "ice_ping"
};
const ::std::string iceC_RoboCompLaser_Laser_getLaserData_name = "getLaserData";
const ::std::string iceC_RoboCompLaser_Laser_getLaserAndBStateData_name = "getLaserAndBStateData";
const ::std::string iceC_RoboCompLaser_Laser_getLaserConfData_name = "getLaserConfData";

}

bool
RoboCompLaser::Laser::ice_isA(::std::string s, const ::Ice::Current&) const
{
    return ::std::binary_search(iceC_RoboCompLaser_Laser_ids, iceC_RoboCompLaser_Laser_ids + 2, s);
}

::std::vector<::std::string>
RoboCompLaser::Laser::ice_ids(const ::Ice::Current&) const
{
    return ::std::vector<::std::string>(&iceC_RoboCompLaser_Laser_ids[0], &iceC_RoboCompLaser_Laser_ids[2]);
}

::std::string
RoboCompLaser::Laser::ice_id(const ::Ice::Current&) const
{
    return ice_staticId();
}

const ::std::string&
RoboCompLaser::Laser::ice_staticId()
{
    static const ::std::string typeId = "::RoboCompLaser::Laser";
    return typeId;
}

/// \cond INTERNAL
bool
RoboCompLaser::Laser::_iceD_getLaserData(::IceInternal::Incoming& inS, const ::Ice::Current& current)
{
    _iceCheckMode(::Ice::OperationMode::Normal, current.mode);
    inS.readEmptyParams();
    TLaserData ret = this->getLaserData(current);
    auto ostr = inS.startWriteParams();
    ostr->writeAll(ret);
    inS.endWriteParams();
    return true;
}
/// \endcond

/// \cond INTERNAL
bool
RoboCompLaser::Laser::_iceD_getLaserAndBStateData(::IceInternal::Incoming& inS, const ::Ice::Current& current)
{
    _iceCheckMode(::Ice::OperationMode::Normal, current.mode);
    inS.readEmptyParams();
    ::RoboCompGenericBase::TBaseState iceP_bState;
    TLaserData ret = this->getLaserAndBStateData(iceP_bState, current);
    auto ostr = inS.startWriteParams();
    ostr->writeAll(iceP_bState, ret);
    inS.endWriteParams();
    return true;
}
/// \endcond

/// \cond INTERNAL
bool
RoboCompLaser::Laser::_iceD_getLaserConfData(::IceInternal::Incoming& inS, const ::Ice::Current& current)
{
    _iceCheckMode(::Ice::OperationMode::Normal, current.mode);
    inS.readEmptyParams();
    LaserConfData ret = this->getLaserConfData(current);
    auto ostr = inS.startWriteParams();
    ostr->writeAll(ret);
    inS.endWriteParams();
    return true;
}
/// \endcond

/// \cond INTERNAL
bool
RoboCompLaser::Laser::_iceDispatch(::IceInternal::Incoming& in, const ::Ice::Current& current)
{
    ::std::pair<const ::std::string*, const ::std::string*> r = ::std::equal_range(iceC_RoboCompLaser_Laser_ops, iceC_RoboCompLaser_Laser_ops + 7, current.operation);
    if(r.first == r.second)
    {
        throw ::Ice::OperationNotExistException(__FILE__, __LINE__, current.id, current.facet, current.operation);
    }

    switch(r.first - iceC_RoboCompLaser_Laser_ops)
    {
        case 0:
        {
            return _iceD_getLaserAndBStateData(in, current);
        }
        case 1:
        {
            return _iceD_getLaserConfData(in, current);
        }
        case 2:
        {
            return _iceD_getLaserData(in, current);
        }
        case 3:
        {
            return _iceD_ice_id(in, current);
        }
        case 4:
        {
            return _iceD_ice_ids(in, current);
        }
        case 5:
        {
            return _iceD_ice_isA(in, current);
        }
        case 6:
        {
            return _iceD_ice_ping(in, current);
        }
        default:
        {
            assert(false);
            throw ::Ice::OperationNotExistException(__FILE__, __LINE__, current.id, current.facet, current.operation);
        }
    }
}
/// \endcond

/// \cond INTERNAL
void
RoboCompLaser::LaserPrx::_iceI_getLaserData(const ::std::shared_ptr<::IceInternal::OutgoingAsyncT<::RoboCompLaser::TLaserData>>& outAsync, const ::Ice::Context& context)
{
    _checkTwowayOnly(iceC_RoboCompLaser_Laser_getLaserData_name);
    outAsync->invoke(iceC_RoboCompLaser_Laser_getLaserData_name, ::Ice::OperationMode::Normal, ::Ice::FormatType::DefaultFormat, context,
        nullptr,
        nullptr);
}
/// \endcond

/// \cond INTERNAL
void
RoboCompLaser::LaserPrx::_iceI_getLaserAndBStateData(const ::std::shared_ptr<::IceInternal::OutgoingAsyncT<Laser::GetLaserAndBStateDataResult>>& outAsync, const ::Ice::Context& context)
{
    _checkTwowayOnly(iceC_RoboCompLaser_Laser_getLaserAndBStateData_name);
    outAsync->invoke(iceC_RoboCompLaser_Laser_getLaserAndBStateData_name, ::Ice::OperationMode::Normal, ::Ice::FormatType::DefaultFormat, context,
        nullptr,
        nullptr,
        [](::Ice::InputStream* istr)
        {
            Laser::GetLaserAndBStateDataResult v;
            istr->readAll(v.bState, v.returnValue);
            return v;
        });
}
/// \endcond

/// \cond INTERNAL
void
RoboCompLaser::LaserPrx::_iceI_getLaserConfData(const ::std::shared_ptr<::IceInternal::OutgoingAsyncT<::RoboCompLaser::LaserConfData>>& outAsync, const ::Ice::Context& context)
{
    _checkTwowayOnly(iceC_RoboCompLaser_Laser_getLaserConfData_name);
    outAsync->invoke(iceC_RoboCompLaser_Laser_getLaserConfData_name, ::Ice::OperationMode::Normal, ::Ice::FormatType::DefaultFormat, context,
        nullptr,
        nullptr);
}
/// \endcond

/// \cond INTERNAL
::std::shared_ptr<::Ice::ObjectPrx>
RoboCompLaser::LaserPrx::_newInstance() const
{
    return ::IceInternal::createProxy<LaserPrx>();
}
/// \endcond

const ::std::string&
RoboCompLaser::LaserPrx::ice_staticId()
{
    return Laser::ice_staticId();
}

namespace Ice
{
}

#else // C++98 mapping

namespace
{

const ::std::string iceC_RoboCompLaser_Laser_getLaserData_name = "getLaserData";

const ::std::string iceC_RoboCompLaser_Laser_getLaserAndBStateData_name = "getLaserAndBStateData";

const ::std::string iceC_RoboCompLaser_Laser_getLaserConfData_name = "getLaserConfData";

}

/// \cond INTERNAL
::IceProxy::Ice::Object* ::IceProxy::RoboCompLaser::upCast(Laser* p) { return p; }

void
::IceProxy::RoboCompLaser::_readProxy(::Ice::InputStream* istr, ::IceInternal::ProxyHandle< Laser>& v)
{
    ::Ice::ObjectPrx proxy;
    istr->read(proxy);
    if(!proxy)
    {
        v = 0;
    }
    else
    {
        v = new Laser;
        v->_copyFrom(proxy);
    }
}
/// \endcond

::Ice::AsyncResultPtr
IceProxy::RoboCompLaser::Laser::_iceI_begin_getLaserData(const ::Ice::Context& context, const ::IceInternal::CallbackBasePtr& del, const ::Ice::LocalObjectPtr& cookie, bool sync)
{
    _checkTwowayOnly(iceC_RoboCompLaser_Laser_getLaserData_name, sync);
    ::IceInternal::OutgoingAsyncPtr result = new ::IceInternal::CallbackOutgoing(this, iceC_RoboCompLaser_Laser_getLaserData_name, del, cookie, sync);
    try
    {
        result->prepare(iceC_RoboCompLaser_Laser_getLaserData_name, ::Ice::Normal, context);
        result->writeEmptyParams();
        result->invoke(iceC_RoboCompLaser_Laser_getLaserData_name);
    }
    catch(const ::Ice::Exception& ex)
    {
        result->abort(ex);
    }
    return result;
}

::RoboCompLaser::TLaserData
IceProxy::RoboCompLaser::Laser::end_getLaserData(const ::Ice::AsyncResultPtr& result)
{
    ::Ice::AsyncResult::_check(result, this, iceC_RoboCompLaser_Laser_getLaserData_name);
    ::RoboCompLaser::TLaserData ret;
    if(!result->_waitForResponse())
    {
        try
        {
            result->_throwUserException();
        }
        catch(const ::Ice::UserException& ex)
        {
            throw ::Ice::UnknownUserException(__FILE__, __LINE__, ex.ice_id());
        }
    }
    ::Ice::InputStream* istr = result->_startReadParams();
    istr->read(ret);
    result->_endReadParams();
    return ret;
}

::Ice::AsyncResultPtr
IceProxy::RoboCompLaser::Laser::_iceI_begin_getLaserAndBStateData(const ::Ice::Context& context, const ::IceInternal::CallbackBasePtr& del, const ::Ice::LocalObjectPtr& cookie, bool sync)
{
    _checkTwowayOnly(iceC_RoboCompLaser_Laser_getLaserAndBStateData_name, sync);
    ::IceInternal::OutgoingAsyncPtr result = new ::IceInternal::CallbackOutgoing(this, iceC_RoboCompLaser_Laser_getLaserAndBStateData_name, del, cookie, sync);
    try
    {
        result->prepare(iceC_RoboCompLaser_Laser_getLaserAndBStateData_name, ::Ice::Normal, context);
        result->writeEmptyParams();
        result->invoke(iceC_RoboCompLaser_Laser_getLaserAndBStateData_name);
    }
    catch(const ::Ice::Exception& ex)
    {
        result->abort(ex);
    }
    return result;
}

::RoboCompLaser::TLaserData
IceProxy::RoboCompLaser::Laser::end_getLaserAndBStateData(::RoboCompGenericBase::TBaseState& iceP_bState, const ::Ice::AsyncResultPtr& result)
{
    ::Ice::AsyncResult::_check(result, this, iceC_RoboCompLaser_Laser_getLaserAndBStateData_name);
    ::RoboCompLaser::TLaserData ret;
    if(!result->_waitForResponse())
    {
        try
        {
            result->_throwUserException();
        }
        catch(const ::Ice::UserException& ex)
        {
            throw ::Ice::UnknownUserException(__FILE__, __LINE__, ex.ice_id());
        }
    }
    ::Ice::InputStream* istr = result->_startReadParams();
    istr->read(iceP_bState);
    istr->read(ret);
    result->_endReadParams();
    return ret;
}

void IceProxy::RoboCompLaser::Laser::_iceI_end_getLaserAndBStateData(::RoboCompGenericBase::TBaseState& iceP_bState, ::RoboCompLaser::TLaserData& ret, const ::Ice::AsyncResultPtr& result)
{
    ::Ice::AsyncResult::_check(result, this, iceC_RoboCompLaser_Laser_getLaserAndBStateData_name);
    if(!result->_waitForResponse())
    {
        try
        {
            result->_throwUserException();
        }
        catch(const ::Ice::UserException& ex)
        {
            throw ::Ice::UnknownUserException(__FILE__, __LINE__, ex.ice_id());
        }
    }
    ::Ice::InputStream* istr = result->_startReadParams();
    istr->read(iceP_bState);
    istr->read(ret);
    result->_endReadParams();
}

::Ice::AsyncResultPtr
IceProxy::RoboCompLaser::Laser::_iceI_begin_getLaserConfData(const ::Ice::Context& context, const ::IceInternal::CallbackBasePtr& del, const ::Ice::LocalObjectPtr& cookie, bool sync)
{
    _checkTwowayOnly(iceC_RoboCompLaser_Laser_getLaserConfData_name, sync);
    ::IceInternal::OutgoingAsyncPtr result = new ::IceInternal::CallbackOutgoing(this, iceC_RoboCompLaser_Laser_getLaserConfData_name, del, cookie, sync);
    try
    {
        result->prepare(iceC_RoboCompLaser_Laser_getLaserConfData_name, ::Ice::Normal, context);
        result->writeEmptyParams();
        result->invoke(iceC_RoboCompLaser_Laser_getLaserConfData_name);
    }
    catch(const ::Ice::Exception& ex)
    {
        result->abort(ex);
    }
    return result;
}

::RoboCompLaser::LaserConfData
IceProxy::RoboCompLaser::Laser::end_getLaserConfData(const ::Ice::AsyncResultPtr& result)
{
    ::Ice::AsyncResult::_check(result, this, iceC_RoboCompLaser_Laser_getLaserConfData_name);
    ::RoboCompLaser::LaserConfData ret;
    if(!result->_waitForResponse())
    {
        try
        {
            result->_throwUserException();
        }
        catch(const ::Ice::UserException& ex)
        {
            throw ::Ice::UnknownUserException(__FILE__, __LINE__, ex.ice_id());
        }
    }
    ::Ice::InputStream* istr = result->_startReadParams();
    istr->read(ret);
    result->_endReadParams();
    return ret;
}

/// \cond INTERNAL
::IceProxy::Ice::Object*
IceProxy::RoboCompLaser::Laser::_newInstance() const
{
    return new Laser;
}
/// \endcond

const ::std::string&
IceProxy::RoboCompLaser::Laser::ice_staticId()
{
    return ::RoboCompLaser::Laser::ice_staticId();
}

RoboCompLaser::Laser::~Laser()
{
}

/// \cond INTERNAL
::Ice::Object* RoboCompLaser::upCast(Laser* p) { return p; }

/// \endcond

namespace
{
const ::std::string iceC_RoboCompLaser_Laser_ids[2] =
{
    "::Ice::Object",
    "::RoboCompLaser::Laser"
};

}

bool
RoboCompLaser::Laser::ice_isA(const ::std::string& s, const ::Ice::Current&) const
{
    return ::std::binary_search(iceC_RoboCompLaser_Laser_ids, iceC_RoboCompLaser_Laser_ids + 2, s);
}

::std::vector< ::std::string>
RoboCompLaser::Laser::ice_ids(const ::Ice::Current&) const
{
    return ::std::vector< ::std::string>(&iceC_RoboCompLaser_Laser_ids[0], &iceC_RoboCompLaser_Laser_ids[2]);
}

const ::std::string&
RoboCompLaser::Laser::ice_id(const ::Ice::Current&) const
{
    return ice_staticId();
}

const ::std::string&
RoboCompLaser::Laser::ice_staticId()
{
#ifdef ICE_HAS_THREAD_SAFE_LOCAL_STATIC
    static const ::std::string typeId = "::RoboCompLaser::Laser";
    return typeId;
#else
    return iceC_RoboCompLaser_Laser_ids[1];
#endif
}

/// \cond INTERNAL
bool
RoboCompLaser::Laser::_iceD_getLaserData(::IceInternal::Incoming& inS, const ::Ice::Current& current)
{
    _iceCheckMode(::Ice::Normal, current.mode);
    inS.readEmptyParams();
    TLaserData ret = this->getLaserData(current);
    ::Ice::OutputStream* ostr = inS.startWriteParams();
    ostr->write(ret);
    inS.endWriteParams();
    return true;
}
/// \endcond

/// \cond INTERNAL
bool
RoboCompLaser::Laser::_iceD_getLaserAndBStateData(::IceInternal::Incoming& inS, const ::Ice::Current& current)
{
    _iceCheckMode(::Ice::Normal, current.mode);
    inS.readEmptyParams();
    ::RoboCompGenericBase::TBaseState iceP_bState;
    TLaserData ret = this->getLaserAndBStateData(iceP_bState, current);
    ::Ice::OutputStream* ostr = inS.startWriteParams();
    ostr->write(iceP_bState);
    ostr->write(ret);
    inS.endWriteParams();
    return true;
}
/// \endcond

/// \cond INTERNAL
bool
RoboCompLaser::Laser::_iceD_getLaserConfData(::IceInternal::Incoming& inS, const ::Ice::Current& current)
{
    _iceCheckMode(::Ice::Normal, current.mode);
    inS.readEmptyParams();
    LaserConfData ret = this->getLaserConfData(current);
    ::Ice::OutputStream* ostr = inS.startWriteParams();
    ostr->write(ret);
    inS.endWriteParams();
    return true;
}
/// \endcond

namespace
{
const ::std::string iceC_RoboCompLaser_Laser_all[] =
{
    "getLaserAndBStateData",
    "getLaserConfData",
    "getLaserData",
    "ice_id",
    "ice_ids",
    "ice_isA",
    "ice_ping"
};

}

/// \cond INTERNAL
bool
RoboCompLaser::Laser::_iceDispatch(::IceInternal::Incoming& in, const ::Ice::Current& current)
{
    ::std::pair<const ::std::string*, const ::std::string*> r = ::std::equal_range(iceC_RoboCompLaser_Laser_all, iceC_RoboCompLaser_Laser_all + 7, current.operation);
    if(r.first == r.second)
    {
        throw ::Ice::OperationNotExistException(__FILE__, __LINE__, current.id, current.facet, current.operation);
    }

    switch(r.first - iceC_RoboCompLaser_Laser_all)
    {
        case 0:
        {
            return _iceD_getLaserAndBStateData(in, current);
        }
        case 1:
        {
            return _iceD_getLaserConfData(in, current);
        }
        case 2:
        {
            return _iceD_getLaserData(in, current);
        }
        case 3:
        {
            return _iceD_ice_id(in, current);
        }
        case 4:
        {
            return _iceD_ice_ids(in, current);
        }
        case 5:
        {
            return _iceD_ice_isA(in, current);
        }
        case 6:
        {
            return _iceD_ice_ping(in, current);
        }
        default:
        {
            assert(false);
            throw ::Ice::OperationNotExistException(__FILE__, __LINE__, current.id, current.facet, current.operation);
        }
    }
}
/// \endcond

/// \cond STREAM
void
RoboCompLaser::Laser::_iceWriteImpl(::Ice::OutputStream* ostr) const
{
    ostr->startSlice(ice_staticId(), -1, true);
    ::Ice::StreamWriter< Laser, ::Ice::OutputStream>::write(ostr, *this);
    ostr->endSlice();
}

void
RoboCompLaser::Laser::_iceReadImpl(::Ice::InputStream* istr)
{
    istr->startSlice();
    ::Ice::StreamReader< Laser, ::Ice::InputStream>::read(istr, *this);
    istr->endSlice();
}
/// \endcond

/// \cond INTERNAL
void
RoboCompLaser::_icePatchObjectPtr(LaserPtr& handle, const ::Ice::ObjectPtr& v)
{
    handle = LaserPtr::dynamicCast(v);
    if(v && !handle)
    {
        IceInternal::Ex::throwUOE(Laser::ice_staticId(), v);
    }
}
/// \endcond

namespace Ice
{
}

#endif
