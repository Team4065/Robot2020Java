// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

//this is almost the unity game engine's vector3 class made in Java
package frc.robot.Utility;


/**
 * Add your docs here.
 */
public class Vector3 {
    public double x, y, z = 0;

    /**shorthand for new Vector3(0, 0, -1)*/
    public static Vector3 back = new Vector3(0, 0, -1);
    /**shorthand for new Vector3(0, -1, 0)*/
    public static Vector3 down = new Vector3(0, -1, 0);
    /**shorthand for new Vector3(0, 0, 1)*/
    public static Vector3 forward = new Vector3(0, 0, 1);
    /**shorthand for new Vector3(-1, 0, 0)*/
    public static Vector3 left = new Vector3(-1, 0, 0);
    /**shorthand for new Vector3(-Infinity, -Infinity, -Infinity)*/
    public static Vector3 negativeInfinity = new Vector3(Double.NEGATIVE_INFINITY, Double.NEGATIVE_INFINITY, Double.NEGATIVE_INFINITY);
    /**shorthand for new Vector3(1, 1, 1)*/
    public static Vector3 one = new Vector3(1, 1, 1);
    /**shorthand for new Vector3(Infinity, Infinity, Infinity)*/
    public static Vector3 positiveInfinity = new Vector3(Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY);
    /**shorthand for new Vector3(1, 0, 0)*/
    public static Vector3 right = new Vector3(1, 0, 0);
    /**shorthand for new Vector3(0, 1, 0)*/
    public static Vector3 up = new Vector3(0, 1, 0);
    /**shorthand for new Vector3(0, 0, 0)*/
    public static Vector3 zero = new Vector3(0, 0, 0);

    

    public Vector3(double _x, double _y, double _z){
        x = _x;
        y = _y;
        z = _z;
    }

    public double magnitude(){
        return Math.sqrt(x * x + y * y + z * z);
    }

    public Vector3 normalized(){
        double magnitude = magnitude();
        if(magnitude > 0)
            return new Vector3(x / magnitude, y / magnitude, z / magnitude);
        else
            return Vector3.zero;
    }

    public double sqrMagnitude(){
        return x * x + y * y + z * z;
    }


    public Vector3 add(Vector3 other){
        return new Vector3(x + other.x, y + other.y, z + other.z);
    }

    public Vector3 sub(Vector3 other){
        return new Vector3(x - other.x, y - other.y, z - other.z);
    }

    public Vector3 mult(double value){
        return new Vector3(x * value, y * value, z * value);
    }

    public Vector3 divide(double value){
        return new Vector3(x / value, y / value, z / value);
    }    

    public void set(double _x, double _y, double _z){
        x = _x;
        y = _y;
        z = _z;
    }

    public String toString(){
        return "x: " + x + " y: " + y + " z: " + z;
    }

    public String toCSV(){
        return x + "," + y + "," + z;
    }


    public static double angle(Vector3 from, Vector3 to){
        double angle = Math.toDegrees(Math.acos(dot(from, to) / (from.magnitude() * to.magnitude())));
        if(Double.isNaN(angle)){
            angle = 0;
        }
        return angle;
    }

    public static double angleInRadians(Vector3 from, Vector3 to){
        double angle = Math.acos(dot(from, to) / (from.magnitude() * to.magnitude()));
        if(Double.isNaN(angle)){
            angle = 0;
        }
        return angle;
    }

    public static Vector3 clampMagnitude(Vector3 vector, double maxLength){
        if(vector.magnitude() > maxLength){
            return vector.normalized().mult(maxLength);
        }else{
            return vector;
        }
    }
    
    public static Vector3 cross(Vector3 a, Vector3 b){
        return new Vector3(
            a.y * b.z - a.z * b.y,
            a.z * b.x - a.x * b.z,
            a.x * b.y - a.y * b.x
        );
    }

    public static double distance(Vector3 a, Vector3 b){
        return a.sub(b).magnitude();
    }

    public static double dot(Vector3 a, Vector3 b){
        return a.x * b.x + a.y * b.y + a.z * b.z;
    }

    public static Vector3 lerp(Vector3 a, Vector3 b, double t){
        if(t > 1)
            t = 1;
        if(t < 0)
            t = 0;
        return a.mult(1 - t).add(b.mult(t));
    }

    public static Vector3 lerpUnclamped(Vector3 a, Vector3 b, double t){
        return a.mult(1 - t).add(b.mult(t));
    }

    public static Vector3 max(Vector3 a, Vector3 b){
        return new Vector3(
            (a.x >= b.x) ? a.x : b.x,
            (a.y >= b.y) ? a.y : b.y,
            (a.z >= b.z) ? a.z : b.z
        );
    }

    public static Vector3 min(Vector3 a, Vector3 b){
        return new Vector3(
            (a.x <= b.x) ? a.x : b.x,
            (a.y <= b.y) ? a.y : b.y,
            (a.z <= b.z) ? a.z : b.z
        );
    }

    public static Vector3 moveTowards(Vector3 current, Vector3 target, double maxDistanceDelta){
        Vector3 temp = target.sub(current);
        if(temp.magnitude() < maxDistanceDelta){
            return target;
        }else{
            return current.add(Vector3.clampMagnitude(temp, maxDistanceDelta));
        }
    }

    public static Vector3 normalize(Vector3 vector){
        double vectorMag = vector.magnitude();
        vector.x /= vectorMag;
        vector.y /= vectorMag;
        vector.z /= vectorMag;
        return vector;
    }

    public static void orthoNotmalize(Vector3 normal, Vector3 tangent){
        Vector3 u1 = normal;
        Vector3 u2 = tangent.sub(Vector3.project(tangent, u1));
        normal = u1.normalized();
        tangent = u2.normalized();
    }

    public static void orthoNotmalize(Vector3 normal, Vector3 tangent, Vector3 binormal){
        Vector3 u1 = normal;
        Vector3 u2 = tangent.sub(Vector3.project(tangent, u1));
        Vector3 u3 = binormal.sub(Vector3.project(binormal, u1)).sub(Vector3.project(binormal, u2));
        normal = u1.normalized();
        tangent = u2.normalized();
        binormal = u3.normalized();
    }

    public static Vector3 project(Vector3 vector, Vector3 onNormal){
        Vector3 output = onNormal.mult(Vector3.dot(vector, onNormal) / onNormal.sqrMagnitude());
        return output;
    }

    public static Vector3 projectOnPlane(Vector3 vector, Vector3 planeNormal){
        return vector.sub(Vector3.project(vector, planeNormal));
    }

    public static Vector3 reflect(Vector3 vector, Vector3 inNormal){
        return vector.sub(Vector3.project(vector, inNormal).mult(2));
    }

    public static Vector3 rotateTowards(Vector3 current, Vector3 target, double maxRadiansDelta, double maxMagnitudeDelta){
        Vector3 temp = target.sub(current);
        double deltaMagnitude = temp.magnitude() - current.magnitude();
        double deltaRadians = angleInRadians(temp, current);

        Vector3 V = current.normalized();
        Vector3 D = target.normalized();
        Vector3 D_tick = Vector3.cross(Vector3.cross(V, D), V).normalized();
        Vector3 Z = V.mult(Math.cos(maxMagnitudeDelta)).add(D_tick.mult(Math.sin(maxMagnitudeDelta)));

        if(deltaMagnitude < maxMagnitudeDelta && deltaRadians < maxMagnitudeDelta){
            return target;
        }else{
            return Z.mult(current.magnitude() + maxMagnitudeDelta * Math.abs(deltaMagnitude) / deltaMagnitude);
        }
    }

    public static Vector3 scale(Vector3 a, Vector3 b){
        return new Vector3(a.x * b.x, a.y * b.y, a.z * b.z);
    }

    public static double signedAngle(Vector3 from, Vector3 to, Vector3 axis){
        double angle = Math.acos(Vector3.dot(from.normalized(), to.normalized()));
        Vector3 cross = Vector3.cross(from, to);
        if (Vector3.dot(axis, cross) > 0) {
            angle = -angle;
        }

        return Math.toDegrees(angle);
    }

    public static double signedAngleInRadians(Vector3 from, Vector3 to, Vector3 axis){
        double angle = angleInRadians(Vector3.projectOnPlane(from, axis), Vector3.projectOnPlane(to, axis));
        double sign = (axis.normalized().sub(Vector3.cross(Vector3.projectOnPlane(from, axis), Vector3.projectOnPlane(to, axis)).normalized()).magnitude() == 0) ? -1 : 1;
        return angle * sign;
    }

    public static Vector3 slerp(Vector3 a, Vector3 b, double t){
        if(t > 1){
            t = 1;
        }
        if(t < 0){
            t = 0;
        }
        double angle = Vector3.angleInRadians(a, b);
        if(angle == 0){
            return a;
        }
        return a.mult(Math.sin((1 - t) * angle) / Math.sin(angle)).add(b.mult(Math.sin(t * angle) / Math.sin(angle)));
    }

    public static Vector3 slerpUnclamped(Vector3 a, Vector3 b, double t){
        double angle = Vector3.angleInRadians(a, b);
        if(angle == 0){
            return a;
        }
        return a.mult(Math.sin((1 - t) * angle) / Math.sin(angle)).add(b.mult(Math.sin(t * angle) / Math.sin(angle)));
    }

    public static Vector3 rotate(Vector3 vector, Vector3 axis, double angle){
        Vector3 _axis = axis.normalized();
        return vector.mult(Math.cos(Math.toRadians(angle))).add(Vector3.cross(vector, _axis).mult(Math.sin(Math.toRadians(angle)))).add(_axis.mult(Vector3.dot(_axis, vector) * (1 - Math.cos(Math.toRadians(angle)))));
    }
}
