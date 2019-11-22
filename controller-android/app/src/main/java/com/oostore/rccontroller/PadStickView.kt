package com.oostore.rccontroller

import android.content.Context
import android.graphics.Canvas
import android.graphics.Color
import android.graphics.Paint
import android.os.Bundle
import android.os.Handler
import android.util.AttributeSet
import android.view.View



class PadStickView @JvmOverloads constructor(context: Context,
                                             attrs: AttributeSet? = null,
                                             defStyleAttr: Int = 0) : View(context,attrs,defStyleAttr){

    private var color: Int = Color.RED
    private var paint: Paint = Paint(Paint.ANTI_ALIAS_FLAG)

    init {
        paint.color = color
    }

    override fun onDraw(canvas: Canvas?) {
        super.onDraw(canvas)
        var radius = Math.min(width,height)/2f //width和height是getWidth()和getHeight()
        canvas?.drawCircle(width/2f,height/2f,radius,paint)
    }
}