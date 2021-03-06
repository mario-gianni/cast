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
// Generated from file `ControllersStatusPrxHelper.java'
//
// Warning: do not edit this file.
//
// </auto-generated>
//

package eu.nifti.env.status;

public final class ControllersStatusPrxHelper extends Ice.ObjectPrxHelperBase implements ControllersStatusPrx
{
    public static ControllersStatusPrx
    checkedCast(Ice.ObjectPrx __obj)
    {
        ControllersStatusPrx __d = null;
        if(__obj != null)
        {
            try
            {
                __d = (ControllersStatusPrx)__obj;
            }
            catch(ClassCastException ex)
            {
                if(__obj.ice_isA(ice_staticId()))
                {
                    ControllersStatusPrxHelper __h = new ControllersStatusPrxHelper();
                    __h.__copyFrom(__obj);
                    __d = __h;
                }
            }
        }
        return __d;
    }

    public static ControllersStatusPrx
    checkedCast(Ice.ObjectPrx __obj, java.util.Map<String, String> __ctx)
    {
        ControllersStatusPrx __d = null;
        if(__obj != null)
        {
            try
            {
                __d = (ControllersStatusPrx)__obj;
            }
            catch(ClassCastException ex)
            {
                if(__obj.ice_isA(ice_staticId(), __ctx))
                {
                    ControllersStatusPrxHelper __h = new ControllersStatusPrxHelper();
                    __h.__copyFrom(__obj);
                    __d = __h;
                }
            }
        }
        return __d;
    }

    public static ControllersStatusPrx
    checkedCast(Ice.ObjectPrx __obj, String __facet)
    {
        ControllersStatusPrx __d = null;
        if(__obj != null)
        {
            Ice.ObjectPrx __bb = __obj.ice_facet(__facet);
            try
            {
                if(__bb.ice_isA(ice_staticId()))
                {
                    ControllersStatusPrxHelper __h = new ControllersStatusPrxHelper();
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

    public static ControllersStatusPrx
    checkedCast(Ice.ObjectPrx __obj, String __facet, java.util.Map<String, String> __ctx)
    {
        ControllersStatusPrx __d = null;
        if(__obj != null)
        {
            Ice.ObjectPrx __bb = __obj.ice_facet(__facet);
            try
            {
                if(__bb.ice_isA(ice_staticId(), __ctx))
                {
                    ControllersStatusPrxHelper __h = new ControllersStatusPrxHelper();
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

    public static ControllersStatusPrx
    uncheckedCast(Ice.ObjectPrx __obj)
    {
        ControllersStatusPrx __d = null;
        if(__obj != null)
        {
            try
            {
                __d = (ControllersStatusPrx)__obj;
            }
            catch(ClassCastException ex)
            {
                ControllersStatusPrxHelper __h = new ControllersStatusPrxHelper();
                __h.__copyFrom(__obj);
                __d = __h;
            }
        }
        return __d;
    }

    public static ControllersStatusPrx
    uncheckedCast(Ice.ObjectPrx __obj, String __facet)
    {
        ControllersStatusPrx __d = null;
        if(__obj != null)
        {
            Ice.ObjectPrx __bb = __obj.ice_facet(__facet);
            ControllersStatusPrxHelper __h = new ControllersStatusPrxHelper();
            __h.__copyFrom(__bb);
            __d = __h;
        }
        return __d;
    }

    public static final String[] __ids =
    {
        "::Ice::Object",
        "::eu::nifti::env::status::ControllersStatus"
    };

    public static String
    ice_staticId()
    {
        return __ids[1];
    }

    protected Ice._ObjectDelM
    __createDelegateM()
    {
        return new _ControllersStatusDelM();
    }

    protected Ice._ObjectDelD
    __createDelegateD()
    {
        return new _ControllersStatusDelD();
    }

    public static void
    __write(IceInternal.BasicStream __os, ControllersStatusPrx v)
    {
        __os.writeProxy(v);
    }

    public static ControllersStatusPrx
    __read(IceInternal.BasicStream __is)
    {
        Ice.ObjectPrx proxy = __is.readProxy();
        if(proxy != null)
        {
            ControllersStatusPrxHelper result = new ControllersStatusPrxHelper();
            result.__copyFrom(proxy);
            return result;
        }
        return null;
    }
}
