using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using UnityEngine;

namespace Assets.OtherDriver
{
    public class SpawnVectors
    {
        public Vector3 _offset;
        public Vector3 _size;
        public Quaternion _rotation;

        public SpawnVectors(float[] offsetArray, float[] sizeArray, float[] rotationArray)
        {
            _size = ArrayToVector3(sizeArray);
            _rotation = Quaternion.identity;
            var rotation = ArrayToVector3(rotationArray);
            _rotation = Quaternion.Euler(rotation);
            _offset = ArrayToVector3(offsetArray);
        }

        public Vector3 ArrayToVector3(float[] array)
        {
            if (array.Length == 3)
            {
                return new Vector3(array[0], array[1], array[2]);
            }

            return new Vector3();
        }
    }
}
