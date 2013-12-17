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
// Generated from file `NodeSeqHelper.java'
//
// Warning: do not edit this file.
//
// </auto-generated>
//

package eu.nifti.env.topograph;

public final class NodeSeqHelper
{
    public static void
    write(IceInternal.BasicStream __os, Node[] __v)
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

    public static Node[]
    read(IceInternal.BasicStream __is)
    {
        Node[] __v;
        final int __len0 = __is.readAndCheckSeqSize(4);
        final String __type0 = Node.ice_staticId();
        __v = new Node[__len0];
        for(int __i0 = 0; __i0 < __len0; __i0++)
        {
            __is.readObject(new IceInternal.SequencePatcher(__v, Node.class, __type0, __i0));
        }
        return __v;
    }
}