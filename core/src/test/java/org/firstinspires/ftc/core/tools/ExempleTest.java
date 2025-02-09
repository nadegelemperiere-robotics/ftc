package org.firstinspires.ftc;

import org.junit.Test;
import static org.junit.Assert.*;

public class ExempleTest {

    @Test
    public void testAddition() {
        int result = 2 + 2;
        assertEquals(4, result);
    }

    @Test
    public void testStringNotEmpty() {
        String str = "FTC Rocks!";
        assertFalse(str.isEmpty());
    }
}
