using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using RoadExtraction.Objects;
using RoadExtraction.Imagery;

namespace RoadExtraction.ImageProcessing
{
    public class BinaryMorphology
    {
        public static byte On = 255;
        public static byte Off = 0;
        public static byte DC = 100;

        #region Binary Image Operations

        public static BinaryImage RotateLeft(BinaryImage Image)
        {
            BinaryImage Output = new BinaryImage(Image.Height, Image.Width);

            for (int i = 0; i < Image.Width; i++)
                for (int j = 0; j < Image.Height; j++)
                    Output[j, Image.Height - i - 1] = Image[i, j];

            return Output;
        }

        public static BinaryImage RotateRight(BinaryImage Image)
        {
            BinaryImage Output = new BinaryImage(Image.Height, Image.Width);

            for (int i = 0; i < Image.Width; i++)
                for (int j = 0; j < Image.Height; j++)
                    Output[ Image.Width-j-1, i] = Image[i, j];

            return Output;
        }

        public static BinaryImage Prune( BinaryImage Image)
        {
            BinaryImage Kernel1 = new BinaryImage(3, 3);
            BinaryImage Kernel2 = new BinaryImage(3, 3);

            Kernel1.Pixels = new byte[] { Off, Off, Off,
                                          Off, On , DC,
                                          Off, Off, DC  };

            Kernel2.Pixels = new byte[] { Off, Off, DC,
                                          Off, On , DC,
                                          Off, Off, Off };

            //allow use of the optimized hit and miss transform for faster speeds
            List<PixelGroup> GroupList = Grouping.GroupPixels(Image, BinaryImage.On);
            PixelGroup PixelObject = PixelGroup.Union(GroupList);

            for (int i = 0; i < 4; i++)
            {
                Image.Remove(HitAndMissTransform(Image, PixelObject, Kernel1, On, Off),false);
                Image.Remove(HitAndMissTransform(Image, PixelObject, Kernel2, On, Off),false);

                Kernel1 = RotateLeft(Kernel1);
                Kernel2 = RotateLeft(Kernel2);
            }

            return Image;
        }

        public static BinaryImage SalientPoints(BinaryImage Image)
        {
            BinaryImage Kernel1 = new BinaryImage(3, 3);
            BinaryImage Kernel2 = new BinaryImage(3, 3);
            BinaryImage Kernel3 = new BinaryImage(3, 3);
            BinaryImage Kernel4 = new BinaryImage(3, 3);

            Kernel1.Pixels = new byte[]{ Off, Off, Off,
                                         On , On , On ,
                                         Off, Off, Off };

            Kernel2.Pixels = new byte[]{ Off, On, Off, 
                                         Off, On, Off,
                                         Off, On, Off  };

            Kernel3.Pixels = new byte[]{ On , Off, Off,
                                         Off, On , Off,
                                         Off, Off, On  };

            Kernel4.Pixels = new byte[]{ Off, Off, On, 
                                         Off, On, Off,
                                         On , Off, Off };

            BinaryImage Output = Image;

            Output = Thinning(Output, Kernel1);
            Output = Thinning(Output, Kernel2);
            Output = Thinning(Output, Kernel3);
            Output = Thinning(Output, Kernel4);

            return Output;
        }

        public static BinaryImage CornerDetector(BinaryImage Image)
        {
            BinaryImage corner1 = new BinaryImage(3, 3);
            BinaryImage corner2 = new BinaryImage(3, 3);
            BinaryImage corner3 = new BinaryImage(3, 3);
            BinaryImage corner4 = new BinaryImage(3, 3);

            corner1.Pixels = new byte[]{ DC,  On,  DC,
                                         Off, On,  On,
                                         Off, Off, DC };

            corner2.Pixels = new byte[]{ DC, On,  DC,
                                         On, On,  Off,
                                         DC, Off, Off };

            corner3.Pixels = new byte[]{ DC, Off, Off,
                                         On, On,  Off,
                                         DC, On,  DC };

            corner4.Pixels = new byte[]{ Off, Off, DC,
                                         Off, On,  On,
                                         DC,  On,  DC };

            BinaryImage CornerSet1 = HitAndMissTransform(Image, corner1, On, Off);
            BinaryImage CornerSet2 = HitAndMissTransform(Image, corner2, On, Off);
            BinaryImage CornerSet3 = HitAndMissTransform(Image, corner3, On, Off);
            BinaryImage CornerSet4 = HitAndMissTransform(Image, corner4, On, Off);

            return CornerSet1 + CornerSet2 + CornerSet3 + CornerSet4;
        }

        public static BinaryImage Skeleton(BinaryImage Image)
        {
            BinaryImage Kernel1 = new BinaryImage(3, 3, new byte[] { Off , DC , On,
                                                                     Off , On , On,
                                                                     Off , DC , On  });

            BinaryImage Kernel2 = new BinaryImage(3, 3, new byte[] { Off , Off, DC,
                                                                     Off , On , On,
                                                                     DC  , On , DC  });

            BinaryImage[] KernelList1 = new BinaryImage[4];
            BinaryImage[] KernelList2 = new BinaryImage[4];

            //allow use of the optimized hit and miss transform for faster speeds
            List<PixelGroup> GroupList = Grouping.GroupPixels(Image, BinaryImage.On);
            PixelGroup PixelObject = PixelGroup.Union(GroupList);

            //store all 4 rotations
            for (int i = 0; i < 4; i++)
            {
                KernelList1[i] = Kernel1;
                Kernel1 = RotateLeft(Kernel1);

                KernelList2[i] = Kernel2;
                Kernel2 = RotateLeft(Kernel2);
            }

            bool changed;

            do
            {
                changed = false;

                for (int i = 0; i < 4; i++)
                {
                    changed = changed | Image.Remove(HitAndMissTransform(Image, PixelObject, KernelList1[i], On, Off),false);
                    changed = changed | Image.Remove(HitAndMissTransform(Image, PixelObject, KernelList2[i], On, Off),false);
                }

            } while (changed);

            return Image;
        }

        public static BinaryImage Thinning(BinaryImage Image, BinaryImage Kernel)
        {
            return Image - HitAndMissTransform(Image, Kernel, On, Off);
        }

        public static BinaryImage BottomHat(BinaryImage Image)
        {
            return Dilation(Image,3) - Image;
        }

        public static BinaryImage TopHat(BinaryImage Image)
        {
            return Image - Erosion(Image,3);
        }

        public static BinaryImage Closing(BinaryImage Image, int KernelSize )
        {
            return Erosion( Dilation(Image,KernelSize), KernelSize );
        }

        public static BinaryImage Opening(BinaryImage Image, int KernelSize )
        {
            return Dilation( Erosion(Image, KernelSize), KernelSize);
        }

        //takes longer on images with a lot of background
        public static BinaryImage Dilation(BinaryImage Image, int KernelSize)
        {
            BinaryImage Kernel = new BinaryImage(KernelSize, KernelSize);

            for (int i = 0; i < Kernel.Width; i++)
                for (int j = 0; j < Kernel.Height; j++)
                    Kernel[i, j] = Off;

            return HitAndMissTransform(Image, Kernel, Off, On);
        }

        //takes longer on images with a lot of foreground
        public static BinaryImage Erosion(BinaryImage Image, int KernelSize)
        {
            BinaryImage Kernel = new BinaryImage(KernelSize, KernelSize);

            for (int i = 0; i < Kernel.Width; i++)
                for (int j = 0; j < Kernel.Height; j++)
                    Kernel[i, j] = On;

            return HitAndMissTransform(Image, Kernel, On, Off);
        }

        //performs a hit and miss transform. Uses a parellelized for loop for best performance
        public static BinaryImage HitAndMissTransform(BinaryImage Image, BinaryImage Kernel, byte Hit, byte Miss )
        {
            int XStart = Convert.ToInt32(-1 * Math.Floor((double)Kernel.Width / 2));
            int XEnd = Convert.ToInt32(Math.Ceiling((double)Kernel.Width / 2));

            int YStart = Convert.ToInt32(-1 * Math.Floor((double)Kernel.Height / 2));
            int YEnd = Convert.ToInt32(Math.Ceiling((double)Kernel.Height / 2));

            BinaryImage Output = new BinaryImage(Image.Width, Image.Height);

            Parallel.For(0, Image.Width, i =>
            {
                Parallel.For(0, Image.Height, j =>
                {
                    //if no group specified or a group was specified and it contains the pixel
                    Boolean miss = false;
                    for (int x = XStart; x < XEnd && !miss; x++)
                    {
                        for (int y = YStart; y < YEnd && !miss; y++)
                        {
                            //check if the pixel in the structuring element is a dont care pixel. If not compare it to the pixel in the Image
                            //check if the pixel is out of bounds
                            if (Kernel[x - XStart, y - YStart] == DC || i + x < 0 || i + x >= Image.Width || j + y < 0 || j + y >= Image.Height )
                                continue;

                            if (Image[i + x, j + y] != Kernel[x - XStart, y - YStart])
                                miss = true;
                        }
                    }

                    Output[i, j] = (miss) ? Miss : Hit;
                });
            });
            return Output;
        }

        //Highly Optimised Hit and Miss Transform that requires the user to specify the region within the image to check
        //NOTE: THIS IS ONLY MEANT FOR THINNING ALGORITHMS LIKE SKELETONIZATION AND PRUNING - MAKE SURE IT IS CLEAR
        private static BinaryImage HitAndMissTransform(BinaryImage Image, PixelGroup PixelObject , BinaryImage Kernel, byte Hit, byte Miss)
        {
            int XStart = Convert.ToInt32(-1 * Math.Floor((double)Kernel.Width / 2));
            int XEnd = Convert.ToInt32(Math.Ceiling((double)Kernel.Width / 2));

            int YStart = Convert.ToInt32(-1 * Math.Floor((double)Kernel.Height / 2));
            int YEnd = Convert.ToInt32(Math.Ceiling((double)Kernel.Height / 2));

            BinaryImage Output = new BinaryImage(Image.Width, Image.Height);

            List<Pixel> PixelList = PixelObject.Pixels.ToList();

            for( int index=0; index<PixelList.Count; index++)
            {
                Pixel pixel = PixelList[index];
                int i = pixel.X;
                int j = pixel.Y;

                //if it is not a border pixel
                if (i != 0 && j != 0 && i != Image.Width - 1 && j != Image.Height - 1)
                {
                    //if no group specified or a group was specified and it contains the pixel
                    Boolean miss = false;
                    for (int x = XStart; x < XEnd && !miss; x++)
                    {
                        for (int y = YStart; y < YEnd && !miss; y++)
                        {
                            //check if the pixel in the structuring element is a dont care pixel. If not compare it to the pixel in the Image
                            //check if the pixel is out of bounds
                            if (Kernel[x - XStart, y - YStart] == DC )
                                continue;

                            if (Image[i + x, j + y] != Kernel[x - XStart, y - YStart])
                                miss = true;
                        }
                    }

                    Output[i, j] = (miss) ? Miss : Hit;
                    if (!miss)
                        PixelObject.RemovePixel(pixel);
                }
                else
                    Output[i, j] = Image[i, j];
            };
            return Output;
        }

        #endregion

        #region Pixel Group Operations

        public static PixelGroup HitAndMissTransform(PixelGroup Group, BinaryImage Kernel, byte Hit, byte Miss)
        {
            int XStart = Convert.ToInt32(-1 * Math.Floor((double)Kernel.Width / 2));
            int XEnd = Convert.ToInt32(Math.Ceiling((double)Kernel.Width / 2));

            int YStart = Convert.ToInt32(-1 * Math.Floor((double)Kernel.Height / 2));
            int YEnd = Convert.ToInt32(Math.Ceiling((double)Kernel.Height / 2));

            PixelGroup Output = new PixelGroup(Group.Width, Group.Height);

            //allow a border from both the start and the end as this is usually requried (example dilation)
            //border is defined by the kernel being used.
            for (int i = Group.MinX+XStart; i <= Group.MaxX+XEnd; i++)
            {
                for (int j = Group.MinY+YStart; j <= Group.MaxY+YEnd; j++)
                {
                    if (i < 0 || j < 0 || i >= Group.Width || j >= Group.Height)
                        continue;

                    //if no group specified or a group was specified and it contains the pixel
                    Boolean miss = false;
                    for (int x = XStart; x < XEnd && !miss; x++)
                    {
                        for (int y = YStart; y < YEnd && !miss; y++)
                        {
                            //check if the pixel in the structuring element is a dont care pixel. If not compare it to the pixel in the Image
                            //check if the pixel is out of bounds
                            if (i + x < 0 || i + x >= Group.Width || j + y < 0 || j + y >= Group.Height || Kernel[x - XStart, y - YStart] == DC)
                                continue;

                            bool Value1 = (Kernel[x - XStart, y - YStart] == BinaryImage.On);
                            bool Value2 = Group.HasPixel(i + x, j + y);

                            if (Value1!=Value2)
                                miss = true;
                        }
                    }

                    bool MissValue = (Miss==BinaryImage.On);
                    bool HitValue = (Hit == BinaryImage.On);

                    if (miss)
                    {
                        if (MissValue)
                            Output.AddPixel(new Pixel(i, j));
                    }
                    else
                    {
                        if (HitValue)
                            Output.AddPixel(new Pixel(i, j));
                    }
                    
                }
            }
            return Output;
        }

        public static PixelGroup TopHat(PixelGroup Group)
        {
            return Group - Erosion(Group, 3);
        }

        public static PixelGroup BottomHat(PixelGroup Group)
        {
            return Dilation(Group, 3) - Group;
        }

        public static PixelGroup Erosion(PixelGroup Group, int KernelSize)
        {
            BinaryImage Kernel = new BinaryImage(KernelSize, KernelSize);
            for (int i = 0; i < KernelSize; i++)
                for (int j = 0; j < KernelSize; j++)
                    Kernel[i, j] = BinaryImage.On;

            PixelGroup Output = new PixelGroup(Group.Width, Group.Height);

            return BinaryMorphology.HitAndMissTransform(Group, Kernel, BinaryImage.On, BinaryImage.Off);
        }

        public static PixelGroup Dilation(PixelGroup Group, int KernelSize)
        {
            BinaryImage Kernel = new BinaryImage(KernelSize, KernelSize);
            for (int i = 0; i < KernelSize; i++)
                for (int j = 0; j < KernelSize; j++)
                    Kernel[i, j] = Off;

            PixelGroup Output = new PixelGroup(Group.Width, Group.Height);

            return BinaryMorphology.HitAndMissTransform(Group, Kernel, Off, On);
        }

        public static PixelGroup Closing(PixelGroup Group, int KernelSize)
        {
            return Erosion(Dilation(Group, KernelSize), KernelSize);
        }

        public static PixelGroup Opening(PixelGroup Group, int KernelSize)
        {
            return Dilation(Erosion(Group, KernelSize), KernelSize);
        }

        #endregion
    }                                                                                        
}
