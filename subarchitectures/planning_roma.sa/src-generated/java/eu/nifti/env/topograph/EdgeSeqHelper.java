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
// Generated from file `EdgeSeqHelper.java'
//
// Warning: do not edit this file.
//
// </auto-generated>
//

package eu.nifti.env.topograph;

public final class EdgeSeqHelper
{
    public static void
    write(IceInternal.BasicStream __os, Edge[] __v)
    {
        if(__v == null)
        {
            __os.writeSize(0);
        }
        else
        {
            __os.writeSize(__v.length);
            for(int __i0 = 0; __i0 < __v.length; __i0++)
            {
                __os.writeObject(__v[__i0]);
            }
        }
    }

    public static Edge[]
    read(IceInternal.BasicStream __is)
    {
        Edge[] __v;
        final int __len0 = __is.readAndCheckSeqSize(4);
        final String __type0 = Edge.ice_staticId();
        __v = new Edge[__len0];
        for(int __i0 = 0; __i0 < __len0; __i0++)
        {
            __is.readObject(new IceInternal.SequencePatcher(__v, Edge.class, __type0, __i0));
        }
        return __v;
    }
}
