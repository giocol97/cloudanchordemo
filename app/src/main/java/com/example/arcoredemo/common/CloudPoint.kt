package com.example.arcoredemo.common

import java.nio.FloatBuffer

class CloudPoint(val x: Float, val y: Float, val z: Float, val confidence: Float=1f, var r:Int=255, var g:Int=255, var b:Int=255) {

    constructor(arr:FloatArray) : this(x=arr[0],y=arr[1],z=arr[2])

    constructor(arr:FloatArray,conf:Float) : this(x=arr[0],y=arr[1],z=arr[2],confidence=conf)

    constructor(arr:FloatArray,conf:Float, red:Int, green:Int, blue:Int) : this(x=arr[0],y=arr[1],z=arr[2],confidence=conf,r=red,g=green, b=blue)

    fun setColor(red:Int, green:Int, blue:Int) {
        r=red
        b=blue
        g=green
    }

    fun toList():List<Float>{
        val list= mutableListOf<Float>()
        list.add(x)
        list.add(y)
        list.add(z)
        return list
    }

    fun toBuffer(): FloatBuffer {
        val buffer=FloatBuffer.allocate(3)//FLOATS_PER_POINT*pointList.size)
        buffer.put(x)
        buffer.put(y)
        buffer.put(z)
        buffer.rewind()
        return buffer
    }

    fun toArray(): FloatArray {
        return floatArrayOf(x, y, z)
    }


}