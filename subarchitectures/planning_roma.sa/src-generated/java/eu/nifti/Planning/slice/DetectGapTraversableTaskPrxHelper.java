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
// Generated from file `DetectGapTraversableTaskPrxHelper.java'
//
// Warning: do not edit this file.
//
// </auto-generated>
//

package eu.nifti.Planning.slice;

public final class DetectGapTraversableTaskPrxHelper extends Ice.ObjectPrxHelperBase implements DetectGapTraversableTaskPrx
{
    public static DetectGapTraversableTaskPrx
    checkedCast(Ice.ObjectPrx __obj)
    {
        DetectGapTraversableTaskPrx __d = null;
        if(__obj != null)
        {
            try
            {
                __d = (DetectGapTraversableTaskPrx)__obj;
            }
            catch(ClassCastException ex)
            {
                if(__obj.ice_isA(ice_staticId()))
                {
                    DetectGapTraversableTaskPrxHelper __h = new DetectGapTraversableTaskPrxHelper();
                    __h.__copyFrom(__obj);
                    __d = __h;
                }
            }
        }
        return __d;
    }

    public static DetectGapTraversableTaskPrx
    checkedCast(Ice.ObjectPrx __obj, java.util.Map<String, String> __ctx)
    {
        DetectGapTraversableTaskPrx __d = null;
        if(__obj != null)
        {
            try
            {
                __d = (DetectGapTraversableTaskPrx)__obj;
            }
            catch(ClassCastException ex)
            {
                if(__obj.ice_isA(ice_staticId(), __ctx))
                {
                    DetectGapTraversableTaskPrxHelper __h = new DetectGapTraversableTaskPrxHelper();
                    __h.__copyFrom(__obj);
                    __d = __h;
                }
            }
        }
        return __d;
    }

    public static DetectGapTraversableTaskPrx
    checkedCast(Ice.ObjectPrx __obj, String __facet)
    {
        DetectGapTraversableTaskPrx __d = null;
        if(__obj != null)
        {
            Ice.ObjectPrx __bb = __obj.ice_facet(__facet);
            try
            {
                if(__bb.ice_isA(ice_staticId()))
                {
                    DetectGapTraversableTaskPrxHelper __h = new DetectGapTraversableTaskPrxHelper();
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

    public static DetectGapTraversableTaskPrx
    checkedCast(Ice.ObjectPrx __obj, String __facet, java.util.Map<String, String> __ctx)
    {
        DetectGapTraversableTaskPrx __d = null;
        if(__obj != null)
        {
            Ice.ObjectPrx __bb = __obj.ice_facet(__facet);
            try
            {
                if(__bb.ice_isA(ice_staticId(), __ctx))
                {
                    DetectGapTraversableTaskPrxHelper __h = new DetectGapTraversableTaskPrxHelper();
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

    public static DetectGapTraversableTaskPrx
    uncheckedCast(Ice.ObjectPrx __obj)
    {
        DetectGapTraversableTaskPrx __d = null;
        if(__obj != null)
        {
            try
            {
                __d = (DetectGapTraversableTaskPrx)__obj;
            }
            catch(ClassCastException ex)
            {
                DetectGapTraversableTaskPrxHelper __h = new DetectGapTraversableTaskPrxHelper();
                __h.__copyFrom(__obj);
                __d = __h;
            }
        }
        return __d;
    }

    public static DetectGapTraversableTaskPrx
    uncheckedCast(Ice.ObjectPrx __obj, String __facet)
    {
        DetectGapTraversableTaskPrx __d = null;
        if(__obj != null)
        {
            Ice.ObjectPrx __bb = __obj.ice_facet(__facet);
            DetectGapTraversableTaskPrxHelper __h = new DetectGapTraversableTaskPrxHelper();
            __h.__copyFrom(__bb);
            __d = __h;
        }
        return __d;
    }

    public static final String[] __ids =
    {
        "::Ice::Object",
        "::eu::nifti::Planning::slice::DetectGapTraversableTask",
        "::eu::nifti::Planning::slice::Task"
    };

    public static String
    ice_staticId()
    {
        return __ids[1];
    }

    protected Ice._ObjectDelM
    __createDelegateM()
    {
        return new _DetectGapTraversableTaskDelM();
    }

    protected Ice._ObjectDelD
    __createDelegateD()
    {
        return new _DetectGapTraversableTaskDelD();
    }

    public static void
    __write(IceInternal.BasicStream __os, DetectGapTraversableTaskPrx v)
    {
        __os.writeProxy(v);
    }

    public static DetectGapTraversableTaskPrx
    __read(IceInternal.BasicStream __is)
    {
        Ice.ObjectPrx proxy = __is.readProxy();
        if(proxy != null)
        {
            DetectGapTraversableTaskPrxHelper result = new DetectGapTraversableTaskPrxHelper();
            result.__copyFrom(proxy);
            return result;
        }
        return null;
    }
}
