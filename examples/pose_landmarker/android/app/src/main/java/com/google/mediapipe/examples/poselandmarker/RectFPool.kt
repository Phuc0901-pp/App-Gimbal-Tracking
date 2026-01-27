package com.google.mediapipe.examples.poselandmarker

import android.graphics.RectF
import java.util.*

/**
 * Object pool for RectF to reduce GC pressure in tracking loop.
 * Pre-allocates RectF objects and reuses them.
 */
object RectFPool {
    private val pool = ArrayDeque<RectF>(20)
    
    init {
        // Pre-allocate 20 RectF objects
        repeat(20) {
            pool.add(RectF())
        }
    }
    
    /**
     * Get a RectF from pool. Creates new one if pool is empty.
     */
    fun obtain(): RectF {
        return pool.pollFirst() ?: RectF()
    }
    
    /**
     * Return a RectF to pool for reuse.
     * Clears the rectangle before adding to pool.
     */
    fun recycle(rect: RectF?) {
        if (rect != null) {
            rect.setEmpty()
            pool.add(rect)
        }
    }
    
    /**
     * Recycle multiple RectF objects at once
     */
    fun recycleAll(rects: Collection<RectF>) {
        rects.forEach { recycle(it) }
    }
    
    /**
     * Get current pool size (for debugging)
     */
    fun getPoolSize(): Int = pool.size
}
