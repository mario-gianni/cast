// **********************************************************************
//
// Copyright (c) 2003-2011 ZeroC, Inc. All rights reserved.
//
// This copy of Ice is licensed to you under the terms described in the
// ICE_LICENSE file included in this distribution.
//
// **********************************************************************
//
// Ice version 3.4.2
//
// <auto-generated>
//
// Generated from file `VantagePointTaskPrxHelper.java'
//
// Warning: do not edit this file.
//
// </auto-generated>
//

package eu.nifti.Planning.slice;

public final class VantagePointTaskPrxHelper extends Ice.ObjectPrxHelperBase implements VantagePointTaskPrx
{
    public static VantagePointTaskPrx
    checkedCast(Ice.ObjectPrx __obj)
    {
        VantagePointTaskPrx __d = null;
        if(__obj != null)
        {
            try
            {
                __d = (VantagePointTaskPrx)__obj;
            }
            catch(ClassCastException ex)
            {
                if(__obj.ice_isA(ice_staticId()))
                {
                    VantagePointTaskPrxHelper __h = new VantagePointTaskPrxHelper();
                    __h.__copyFrom(__obj);
                    __d = __h;
                }
            }
        }
        return __d;
    }

    public static VantagePointTaskPrx
    checkedCast(Ice.ObjectPrx __obj, java.util.Map<String, String> __ctx)
    {
        VantagePointTaskPrx __d = null;
        if(__obj != null)
        {
            try
            {
                __d = (VantagePointTaskPrx)__obj;
            }
            catch(ClassCastException ex)
            {
                if(__obj.ice_isA(ice_staticId(), __ctx))
                {
                    VantagePointTaskPrxHelper __h = new VantagePointTaskPrxHelper();
                    __h.__copyFrom(__obj);
                    __d = __h;
                }
            }
        }
        return __d;
    }

    public static VantagePointTaskPrx
    checkedCast(Ice.ObjectPrx __obj, String __facet)
    {
        VantagePointTaskPrx __d = null;
        if(__obj != null)
        {
            Ice.ObjectPrx __bb = __obj.ice_facet(__facet);
            try
            {
                if(__bb.ice_isA(ice_staticId()))
                {
                    VantagePointTaskPrxHelper __h = new VantagePointTaskPrxHelper();
                    __h.__copyFrom(__bb);
                    __d = __h;
                }
            }
            catch(Ice.FacetNotExistException ex)
            {
            }
        }
        return __d;
    }

    public static VantagePointTaskPrx
    checkedCast(Ice.ObjectPrx __obj, String __facet, java.util.Map<String, String> __ctx)
    {
        VantagePointTaskPrx __d = null;
        if(__obj != null)
        {
            Ice.ObjectPrx __bb = __obj.ice_facet(__facet);
            try
            {
                if(__bb.ice_isA(ice_staticId(), __ctx))
                {
                    VantagePointTaskPrxHelper __h = new VantagePointTaskPrxHelper();
                    __h.__copyFrom(__bb);
                    __d = __h;
                }
            }
            catch(Ice.FacetNotExistException ex)
            {
            }
        }
        return __d;
    }

    public static VantagePointTaskPrx
    uncheckedCast(Ice.ObjectPrx __obj)
    {
        VantagePointTaskPrx __d = null;
        if(__obj != null)
        {
            try
            {
                __d = (VantagePointTaskPrx)__obj;
            }
            catch(ClassCastException ex)
            {
                VantagePointTaskPrxHelper __h = new VantagePointTaskPrxHelper();
                __h.__copyFrom(__obj);
                __d = __h;
            }
        }
        return __d;
    }

    public static VantagePointTaskPrx
    uncheckedCast(Ice.ObjectPrx __obj, String __facet)
    {
        VantagePointTaskPrx __d = null;
        if(__obj != null)
        {
            Ice.ObjectPrx __bb = __obj.ice_facet(__facet);
            VantagePointTaskPrxHelper __h = new VantagePointTaskPrxHelper();
            __h.__copyFrom(__bb);
            __d = __h;
        }
        return __d;
    }

    public static final String[] __ids =
    {
        "::Ice::Object",
        "::eu::nifti::Planning::slice::Task",
        "::eu::nifti::Planning::slice::VantagePointTask"
    };

    public static String
    ice_staticId()
    {
        return __ids[2];
    }

    protected Ice._ObjectDelM
    __createDelegateM()
    {
        return new _VantagePointTaskDelM();
    }

    protected Ice._ObjectDelD
    __createDelegateD()
    {
        return new _VantagePointTaskDelD();
    }

    public static void
    __write(IceInternal.BasicStream __os, VantagePointTaskPrx v)
    {
        __os.writeProxy(v);
    }

    public static VantagePointTaskPrx
    __read(IceInternal.BasicStream __is)
    {
        Ice.ObjectPrx proxy = __is.readProxy();
        if(proxy != null)
        {
            VantagePointTaskPrxHelper result = new VantagePointTaskPrxHelper();
            result.__copyFrom(proxy);
            return result;
        }
        return null;
    }
}
