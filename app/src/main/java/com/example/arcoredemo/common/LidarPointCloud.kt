package com.example.arcoredemo.common

import android.util.Log
import com.google.ar.core.Pose
import java.nio.FloatBuffer
import kotlin.math.cos
import kotlin.math.sin

class LidarPointCloud(var angleOffset: Int) :CloudInterface{

    private val FLOATS_PER_POINT = 3 // X,Y,Z


    val pointList= mutableListOf<LidarPoint>()
    val pointArray=Array<LidarPoint>(360){i->
        LidarPoint(i,-1f,-1)


    }
    var posedPointList=mutableListOf<Float>()
    var lastTimestamp: Long? = null
    lateinit var lastPose: Pose
    lateinit var firstPose:Pose
    var counter=0


    override fun getTimestamp():Long{
        return lastTimestamp!!
    }

    override fun getPoints():FloatBuffer{
        /*if(counter.mod(100)==0 || counter==0){//ogni 200 pose aggiungo dei punti da visualizzare
            pointList.forEach { pt ->
                val arr=firstPose.transformPoint(pt.toCartesian(angleOffset))
                arr.forEach {
                    posedPointList.add(it)
                }
            }
        }*/
        posedPointList=mutableListOf<Float>()

        pointArray.forEach {pt->
            if(pt.distance>0){
                val arr=firstPose.transformPoint(pt.toCartesian(angleOffset))
                arr.forEach {
                    posedPointList.add(it)
                }
            }
        }


        val buffer=FloatBuffer.allocate(posedPointList.size)//FLOATS_PER_POINT*pointList.size)

        posedPointList.forEach { pt ->
            buffer.put(pt)
        }
        buffer.rewind()
        return buffer
    }

    fun addPoint(pt:LidarPoint){
        pointList.add(pt)
    }

    fun addPoints(pts:Array<LidarPoint>){
        pts.forEach { pt ->
            if(pt.angle!=-1){
                pointArray[pt.angle]=pt
                //pointList.add(pt)
            }

        }
    }

    fun setPose(p:Pose){
        counter++
        if(!::firstPose.isInitialized){
            firstPose=p
        }
        if(counter.mod(100)==0){
            firstPose=p
        }
        lastPose=p
    }

}

data class LidarPoint(val angle: Int,val distance: Float, val intensity: Int){
    fun toCartesian(angleOffset: Int=0): FloatArray {
        val angleRads = (angle+angleOffset) * Math.PI / 180

        //distance expressed in meters, height set to -40cm from phone
        return floatArrayOf((distance* cos(angleRads)/1000).toFloat(),-0.4f,(distance* sin(angleRads)/1000).toFloat())//,intensity.toFloat())
    }
}