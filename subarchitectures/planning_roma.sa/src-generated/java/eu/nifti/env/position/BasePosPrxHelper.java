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
// Generated from file `BasePosPrxHelper.java'
//
// Warning: do not edit this file.
//
// </auto-generated>
//

package eu.nifti.env.position;

public final class BasePosPrxHelper extends Ice.ObjectPrxHelperBase implements BasePosPrx
{
    public static BasePosPrx
    checkedCast(Ice.ObjectPrx __obj)
    {
        BasePosPrx __d = null;
        if(__obj != null)
        {
            try
            {
                __d = (BasePosPrx)__obj;
            }
            catch(ClassCastException ex)
            {
                if(__obj.ice_isA(ice_staticId()))
                {
                    BasePosPrxHelper __h = new BasePosPrxHelper();
                    __h.__copyFrom(__obj);
                    __d = __h;
                }
            }
        }
        return __d;
    }

    public static BasePosPrx
    checkedCast(Ice.ObjectPrx __obj, java.util.Map<String, String> __ctx)
    {
        BasePosPrx __d = null;
        if(__obj != null)
        {
            try
            {
                __d = (BasePosPrx)__obj;
            }
            catch(ClassCastException ex)
            {
                if(__obj.ice_isA(ice_staticId(), __ctx))
                {
                    BasePosPrxHelper __h = new BasePosPrxHelper();
                    __h.__copyFrom(__obj);
                    __d = __h;
                }
            }
        }
        return __d;
    }

    public static BasePosPrx
    checkedCast(Ice.ObjectPrx __obj, String __facet)
    {
        BasePosPrx __d = null;
        if(__obj != null)
        {
            Ice.ObjectPrx __bb = __obj.ice_facet(__facet);
            try
            {
                if(__bb.ice_isA(ice_staticId()))
                {
                    BasePosPrxHelper __h = new BasePosPrxHelper();
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

    public static BasePosPrx
    checkedCast(Ice.ObjectPrx __obj, String __facet, java.util.Map<String, String> __ctx)
    {
        BasePosPrx __d = null;
        if(__obj != null)
        {
            Ice.ObjectPrx __bb = __obj.ice_facet(__facet);
            try
            {
                if(__bb.ice_isA(ice_staticId(), __ctx))
                {
                    BasePosPrxHelper __h = new BasePosPrxHelper();
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

    public static BasePosPrx
    uncheckedCast(Ice.ObjectPrx __obj)
    {
        BasePosPrx __d = null;
        if(__obj != null)
        {
            try
            {
                __d = (BasePosPrx)__obj;
            }
            catch(ClassCastException ex)
            {
                BasePosPrxHelper __h = new BasePosPrxHelper();
                __h.__copyFrom(__obj);
                __d = __h;
            }
        }
        return __d;
    }

    public static BasePosPrx
    uncheckedCast(Ice.ObjectPrx __obj, String __facet)
    {
        BasePosPrx __d = null;
        if(__obj != null)
        {
            Ice.ObjectPrx __bb = __obj.ice_facet(__facet);
            BasePosPrxHelper __h = new BasePosPrxHelper();
            __h.__copyFrom(__bb);
            __d = __h;
        }
        return __d;
    }

    public static final String[] __ids =
    {
        "::Ice::Object",
        "::eu::nifti::env::position::BasePos"
    };

    public static String
    ice_staticId()
    {
        return __ids[1];
    }

    protected Ice._ObjectDelM
    __createDelegateM()
    {
        return new _BasePosDelM();
    }

    protected Ice._ObjectDelD
    __createDelegateD()
    {
        return new _BasePosDelD();
    }

    public static void
    __write(IceInternal.BasicStream __os, BasePosPrx v)
    {
        __os.writeProxy(v);
    }

    public static BasePosPrx
    __read(IceInternal.BasicStream __is)
    {
        Ice.ObjectPrx proxy = __is.readProxy();
        if(proxy != null)
        {
            BasePosPrxHelper result = new BasePosPrxHelper();
            result.__copyFrom(proxy);
            return result;
        }
        return null;
    }
}