package com.example.arcoredemo.common

import java.nio.FloatBuffer

interface CloudInterface {

    fun getTimestamp():Long

    fun getPoints(): FloatBuffer
}