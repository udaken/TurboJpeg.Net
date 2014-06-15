using System;
using System.Collections.Generic;
using System.Drawing;
using System.Runtime.InteropServices;
using TurboJpeg.Internal;
using PixelFormat = System.Drawing.Imaging.PixelFormat;

namespace TurboJpeg
{
    public enum TJSamp : int
    {
        Samp444 = 0,
        Samp422,
        Samp420,
        SampGray,
        Samp440,

        Max,
    }
    public enum TJPixelFormat : int
    {
        PixcelFormatRGB = 0,
        PixcelFormatBGR,
        PixcelFormatRGBX,
        PixcelFormatBGRX,
        PixcelFormatXBGR,
        PixcelFormatXRGB,
        PixcelFormatGRAY,
        PixcelFormatRGBA,
        PixcelFormatBGRA,
        PixcelFormatABGR,
        PixcelFormatARGB,

        Max,
    }
    [Flags]
    public enum TJFlag : int
    {
        None = 0,
        BottomUp = TJ.FLAG_BOTTOMUP,
        ForceMMX = TJ.FLAG_FORCEMMX,
        ForceSSE = TJ.FLAG_FORCESSE,
        ForceSSE2 = TJ.FLAG_FORCESSE2,
        ForceSSE3 = TJ.FLAG_FORCESSE3,
        ForceFastUpSample = TJ.FLAG_FASTUPSAMPLE,
        FastDct = TJ.FLAG_FASTDCT,
        AccurateDct = TJ.FLAG_ACCURATEDCT,
    }
    public static class TJ
    {
        public static void CheckSubsampling(TJSamp subsamp)
        {
            if (subsamp < 0 || subsamp >= TJSamp.Max)
                throw new ArgumentException("Invalid subsampling type");
        }
        public static int getMCUWidth(TJSamp subsamp)
        {
            CheckSubsampling(subsamp);
            return mcuWidth[(int)subsamp];
        }

        private static readonly int[] mcuWidth =
        {
            8, 16, 16, 8, 8
          };

        public static int getMCUHeight(TJSamp subsamp)
        {
            CheckSubsampling(subsamp);
            return mcuHeight[(int)subsamp];
        }

        private static readonly int[] mcuHeight =
        {
            8, 8, 16, 8, 16
        };


        internal static void CheckPixelFormat(TJPixelFormat pixelFormat, string message)
        {
            if (pixelFormat < 0 || pixelFormat >= TJPixelFormat.Max)
                throw new ArgumentException(message);
        }
        public static void CheckPixelFormat(TJPixelFormat pixelFormat)
        {
            CheckPixelFormat(pixelFormat, "Invalid pixel format");
        }
        public static int getPixelSize(TJPixelFormat pixelFormat)
        {
            CheckPixelFormat(pixelFormat);
            return pixelSize[(int)pixelFormat];
        }

        private static readonly int[] pixelSize = 
        {
            3, 3, 4, 4, 4, 4, 1, 4, 4, 4, 4
          };

        public static int getRedOffset(TJPixelFormat pixelFormat)
        {
            CheckPixelFormat(pixelFormat);
            return redOffset[(int)pixelFormat];
        }

        private static readonly int[] redOffset = 
        {
            0, 2, 0, 2, 3, 1, 0, 0, 2, 3, 1
          };

        public static int getGreenOffset(TJPixelFormat pixelFormat)
        {
            CheckPixelFormat(pixelFormat);
            return greenOffset[(int)pixelFormat];
        }

        private static readonly int[] greenOffset = 
        {
            1, 1, 1, 1, 2, 2, 0, 1, 1, 2, 2
          };

        public static int getBlueOffset(TJPixelFormat pixelFormat)
        {
            CheckPixelFormat(pixelFormat);
            return blueOffset[(int)pixelFormat];
        }

        private static readonly int[] blueOffset =
        {
            2, 0, 2, 0, 1, 3, 0, 2, 0, 1, 3
          };

        public const int FLAG_BOTTOMUP = 2;
        public const int FLAG_FORCEMMX = 8;
        public const int FLAG_FORCESSE = 16;
        public const int FLAG_FORCESSE2 = 32;
        public const int FLAG_FORCESSE3 = 128;
        public const int FLAG_FASTUPSAMPLE = 256;
        public const int FLAG_FASTDCT = 2048;
        public const int FLAG_ACCURATEDCT = 4096;

        public static int BufSize(int width, int height, TJSamp jpegSubsamp)
        {
            var ret = (int)tjBufSize(width, height, (int)jpegSubsamp);
            if (ret == -1)
            {
                throw new TJException(tjGetErrorStr());
            }
            return ret;
        }
        public static int BufSizeYUV(int width, int height, TJSamp subsamp)
        {
            var ret = (int)tjBufSizeYUV(width, height, (int)subsamp);
            if (ret == -1)
            {
                throw new TJException(tjGetErrorStr());
            }
            return ret;
        }
        public static TJScalingFactor[] GetScalingFactors()
        {
            int n;
            var sf = TJ.tjGetScalingFactors(out n);
            var sfArray = new TJScalingFactor[n];
            for (int i = 0; i < n; i++)
            {
                sfArray[i] = (TJScalingFactor)Marshal.PtrToStructure(new IntPtr(sf.ToInt64() + i * Marshal.SizeOf(typeof(TJScalingFactor))), typeof(TJScalingFactor));
            }
            return sfArray;
        }

        [DllImport("turbojpeg.dll", CallingConvention = CallingConvention.Cdecl)]
        public static extern tjhandle tjInitDecompress();

        [DllImport("turbojpeg.dll", CallingConvention = CallingConvention.Cdecl)]
        public static extern uint tjBufSize(int width, int height, int subsamp);

        [DllImport("turbojpeg.dll", CallingConvention = CallingConvention.Cdecl)]
        public static extern uint tjBufSizeYUV(int width, int height, int subsamp);

        [DllImport("turbojpeg.dll", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        [return: MarshalAs(UnmanagedType.LPStr)]
        public static extern String tjGetErrorStr();

        [DllImport("turbojpeg.dll", CallingConvention = CallingConvention.Cdecl)]
        public static extern IntPtr tjGetScalingFactors(out int numscalingfactors);

        [DllImport("turbojpeg.dll", CallingConvention = CallingConvention.Cdecl)]
        public static extern int tjDestroy(IntPtr handle);
    }
    class TJException : Exception
    {
        public TJException(string message)
            : base(message)
        {
        }
    }
    [StructLayout(LayoutKind.Sequential)]
    public class TJScalingFactor : IEquatable<TJScalingFactor>
    {
        TJScalingFactor()
        {
        }
        public TJScalingFactor(int num, int denom)
        {
            if (num < 1 || denom < 1)
                throw new ArgumentException("Numerator and denominator must be >= 1");
            this.num = num;
            this.denom = denom;
        }

        public int Num
        {
            get
            {
                return num;
            }
        }

        public int Denom
        {
            get
            {
                return denom;
            }
        }

        public int GetScaled(int dimension)
        {
            return (dimension * num + denom - 1) / denom;
        }

        public bool Equals(TJScalingFactor other)
        {
            return (this.num == other.num && this.denom == other.denom);
        }

        public bool IsOne
        {
            get
            {
                return (num == 1 && denom == 1);
            }
        }
        public override string ToString()
        {
            return "num:" + num + ", denom:" + denom;
        }
        private int num = 1;
        private int denom = 1;
    };
    namespace Internal
    {
        public sealed class tjhandle : System.Runtime.InteropServices.SafeHandle
        {
            public tjhandle()
                : base(IntPtr.Zero, true)
            {
            }
            public override bool IsInvalid
            {
                get { return handle == IntPtr.Zero; }
            }

            protected override bool ReleaseHandle()
            {
                return TJ.tjDestroy(handle) != -1;
            }
        }
    }
    public class TJDecompressor : IDisposable
    {
        private const String NO_ASSOC_ERROR =
          "No JPEG image is associated with this instance";

        public TJDecompressor()
        {
            handle = TJ.tjInitDecompress();
            if (handle.IsInvalid)
                throw new TJException("executing tjInitDecompress()");
        }

        public TJDecompressor(byte[] jpegImage)
            : this(jpegImage, jpegImage.Length)
        {
        }

        public TJDecompressor(byte[] jpegImage, int imageSize)
            : this()
        {
            SetJPEGImage(jpegImage, imageSize);
        }
        public TJDecompressor(string filename)
            : this(System.IO.File.OpenRead(filename))
        {
        }
        public TJDecompressor(System.IO.Stream stream)
            : this()
        {
            SetJPEGImage(stream);
        }
        public void SetJPEGImage(System.IO.Stream stream)
        {
            if(!TrySetJPEGImage(stream))
            {
                throw new TJException("Error in tjDecompressHeader2()");
            }
        }
        public bool TrySetJPEGImage(System.IO.Stream stream)
        {
            var memStream = stream as System.IO.MemoryStream;
            if (memStream != null)
            {
                return TrySetJPEGImage(memStream.GetBuffer(), (int)memStream.Length);
            }
            else
            {
                var buf = new byte[stream.Length];
                stream.Read(buf, 0, buf.Length);
                return TrySetJPEGImage(buf, buf.Length);
            }
        }
        public bool TrySetJPEGImage(byte[] jpegImage, int imageSize)
        {
            if (jpegImage == null || imageSize < 1)
                throw new ArgumentException("Invalid argument in setJPEGImage()");

            if (jpegImage.Length < imageSize)
                throw new ArgumentOutOfRangeException();

            int width = 0, height = 0, jpegSubsamp = -1;
            if (tjDecompressHeader2(handle, jpegImage, (uint)imageSize, out width, out height, out jpegSubsamp) == -1)
            {
                return false;
            }
            this.jpegBuf = jpegImage;
            this.jpegBufSize = imageSize;
            this.jpegSubsamp = (TJSamp)jpegSubsamp;
            this.jpegWidth = width;
            this.jpegHeight = height;
            return true;
        }
        public void SetJPEGImage(byte[] jpegImage, int imageSize)
        {
            if (!TrySetJPEGImage(jpegImage, imageSize))
            {
                throw new TJException("Error in tjDecompressHeader2()");
            }
        }

        public int Width
        {
            get
            {
                if (jpegWidth < 1)
                    throw new InvalidOperationException(NO_ASSOC_ERROR);
                return jpegWidth;
            }
        }

        public int Height
        {
            get
            {
                if (jpegHeight < 1)
                    throw new InvalidOperationException(NO_ASSOC_ERROR);
                return jpegHeight;
            }
        }

        public TJSamp Subsamp
        {
            get
            {
                if (jpegSubsamp < 0)
                    throw new InvalidOperationException(NO_ASSOC_ERROR);
                if (jpegSubsamp >= TJSamp.Max)
                    throw new Exception("JPEG header information is invalid");
                return jpegSubsamp;
            }
        }
        public int BufSize
        {
            get
            {
                return TJ.BufSize(Width, Height, Subsamp);
            }
        }

        public byte[] JpegBuf
        {
            get
            {
                if (jpegBuf == null)
                    throw new InvalidOperationException(NO_ASSOC_ERROR);
                return jpegBuf;
            }
        }

        public int JpegSize
        {
            get
            {
                if (jpegBufSize < 1)
                    throw new InvalidOperationException(NO_ASSOC_ERROR);
                return jpegBufSize;
            }
        }

        public int GetScaledWidth(int desiredWidth, int desiredHeight)
        {
            if (jpegWidth < 1 || jpegHeight < 1)
                throw new InvalidOperationException(NO_ASSOC_ERROR);
            if (desiredWidth < 0 || desiredHeight < 0)
                throw new Exception("Invalid argument in getScaledWidth()");
            TJScalingFactor[] sf = TJ.GetScalingFactors();
            if (desiredWidth == 0)
                desiredWidth = jpegWidth;
            if (desiredHeight == 0)
                desiredHeight = jpegHeight;
            int scaledWidth = jpegWidth, scaledHeight = jpegHeight;
            for (int i = 0; i < sf.Length; i++)
            {
                scaledWidth = sf[i].GetScaled(jpegWidth);
                scaledHeight = sf[i].GetScaled(jpegHeight);
                if (scaledWidth <= desiredWidth && scaledHeight <= desiredHeight)
                    break;
            }
            if (scaledWidth > desiredWidth || scaledHeight > desiredHeight)
                throw new Exception("Could not scale down to desired image dimensions");
            return scaledWidth;
        }

        public int GetScaledHeight(int desiredWidth, int desiredHeight)
        {
            if (jpegWidth < 1 || jpegHeight < 1)
                throw new InvalidOperationException(NO_ASSOC_ERROR);
            if (desiredWidth < 0 || desiredHeight < 0)
                throw new ArgumentException("Invalid argument in getScaledHeight()");
            TJScalingFactor[] sf = TJ.GetScalingFactors();
            if (desiredWidth == 0)
                desiredWidth = jpegWidth;
            if (desiredHeight == 0)
                desiredHeight = jpegHeight;
            int scaledWidth = jpegWidth, scaledHeight = jpegHeight;
            for (int i = 0; i < sf.Length; i++)
            {
                scaledWidth = sf[i].GetScaled(jpegWidth);
                scaledHeight = sf[i].GetScaled(jpegHeight);
                if (scaledWidth <= desiredWidth && scaledHeight <= desiredHeight)
                    break;
            }
            if (scaledWidth > desiredWidth || scaledHeight > desiredHeight)
                throw new Exception("Could not scale down to desired image dimensions");
            return scaledHeight;
        }
        private void CheckDecompressArgument(int x, int y, int desiredWidth, int pitch, int desiredHeight, TJPixelFormat pixelFormat, TJFlag flags)
        {
        }

        public void Decompress(IntPtr dstBuf, int x, int y, int desiredWidth, int pitch, int desiredHeight, TJPixelFormat pixelFormat, TJFlag flags)
        {
            if (jpegBuf == null)
                throw new InvalidOperationException(NO_ASSOC_ERROR);
            if (dstBuf == null || x < 0 || y < 0 || desiredWidth < 0 || pitch < 0 ||
                desiredHeight < 0 || flags < 0)
                throw new ArgumentException("Invalid argument in decompress()");

            TJ.CheckPixelFormat(pixelFormat, "Invalid argument in decompress()");

            if (x > 0 || y > 0)
                DecompressInternal(jpegBuf, jpegBufSize, dstBuf, x, y, desiredWidth, pitch,
                           desiredHeight, pixelFormat, flags);
            else
                DecompressInternal(jpegBuf, jpegBufSize, dstBuf, 0, 0, desiredWidth, pitch,
                           desiredHeight, pixelFormat, flags);
        }
        public void Decompress(byte[] dstBuf, int x, int y, int desiredWidth, int pitch, int desiredHeight, TJPixelFormat pixelFormat, TJFlag flags)
        {
            if (jpegBuf == null)
                throw new InvalidOperationException(NO_ASSOC_ERROR);
            if (dstBuf == null || x < 0 || y < 0 || desiredWidth < 0 || pitch < 0 ||
                desiredHeight < 0 || flags < 0)
                throw new ArgumentException("Invalid argument in decompress()");

            TJ.CheckPixelFormat(pixelFormat, "Invalid argument in decompress()");

            if (x > 0 || y > 0)
                DecompressInternal(jpegBuf, jpegBufSize, dstBuf, x, y, desiredWidth, pitch,
                           desiredHeight, pixelFormat, flags);
            else
                DecompressInternal(jpegBuf, jpegBufSize, dstBuf, 0, 0, desiredWidth, pitch,
                           desiredHeight, pixelFormat, flags);
        }
        public void Decompress(int[] dstBuf, int x, int y, int desiredWidth, int stride, int desiredHeight, TJPixelFormat pixelFormat, TJFlag flags)
        {
            if (jpegBuf == null)
                throw new InvalidOperationException(NO_ASSOC_ERROR);
            if (dstBuf == null || x < 0 || y < 0 || desiredWidth < 0 || stride < 0 ||
                desiredHeight < 0 || flags < 0)
                throw new ArgumentException("Invalid argument in decompress()");

            TJ.CheckPixelFormat(pixelFormat, "Invalid argument in decompress()");

            DecompressInternal(jpegBuf, jpegBufSize, dstBuf, x, y, desiredWidth, stride,
                       desiredHeight, pixelFormat, flags);
        }

        public byte[] Decompress(int desiredWidth, int pitch, int desiredHeight, TJPixelFormat pixelFormat, TJFlag flags)
        {
            if (desiredWidth < 0 || pitch < 0 || desiredHeight < 0 || flags < 0)
                throw new ArgumentException("Invalid argument in decompress()");

            TJ.CheckPixelFormat(pixelFormat, "Invalid argument in decompress()");

            int pixelSize = TJ.getPixelSize(pixelFormat);
            int scaledWidth = GetScaledWidth(desiredWidth, desiredHeight);
            int scaledHeight = GetScaledHeight(desiredWidth, desiredHeight);
            if (pitch == 0)
                pitch = scaledWidth * pixelSize;
            byte[] buf = new byte[pitch * scaledHeight];
            Decompress(buf, 0, 0, desiredWidth, pitch, desiredHeight, pixelFormat, flags);
            return buf;
        }

        public void DecompressToYUV(byte[] dstBuf, TJFlag flags)
        {
            if (jpegBuf == null)
                throw new InvalidOperationException(NO_ASSOC_ERROR);
            if (dstBuf == null || flags < 0)
                throw new ArgumentException("Invalid argument in decompressToYUV()");
            DecompressToYUV(jpegBuf, jpegBufSize, dstBuf, flags);
        }


        public byte[] DecompressToYUV(TJFlag flags)
        {
            if (flags < 0)
                throw new ArgumentException("Invalid argument in decompressToYUV()");
            if (jpegWidth < 1 || jpegHeight < 1 || jpegSubsamp < 0)
                throw new InvalidOperationException(NO_ASSOC_ERROR);
            if (jpegSubsamp >= TJSamp.Max)
                throw new Exception("JPEG header information is invalid");
            byte[] buf = new byte[TJ.BufSizeYUV(jpegWidth, jpegHeight, jpegSubsamp)];
            DecompressToYUV(buf, flags);
            return buf;
        }

        public static Bitmap DecompressToBitmap(byte[] srcBuf, TJFlag flags)
        {
            using (var decompressor = new TJDecompressor(srcBuf))
            {
                var pf = decompressor.Subsamp == TJSamp.SampGray ? PixelFormat.Indexed : PixelFormat.Format24bppRgb;
                var dstImage = new Bitmap(decompressor.Width, decompressor.jpegHeight, pf);

                decompressor.Decompress(dstImage, flags);
                return dstImage;
            }
        }
        public void Decompress(Bitmap dstImage, TJFlag flags)
        {
            if (dstImage == null || flags < 0)
                throw new ArgumentException("Invalid argument in decompress()");
            int desiredWidth = dstImage.Width;
            int desiredHeight = dstImage.Height;
            int scaledWidth = GetScaledWidth(desiredWidth, desiredHeight);
            int scaledHeight = GetScaledHeight(desiredWidth, desiredHeight);
            if (scaledWidth != desiredWidth || scaledHeight != desiredHeight)
                throw new Exception("Bitmap dimensions do not match one of the scaled image sizes that TurboJPEG is capable of generating.");
            TJPixelFormat pixelFormat;
            bool intPixels = false;

            switch (dstImage.PixelFormat)
            {
                case PixelFormat.Format24bppRgb:
                    pixelFormat = TJPixelFormat.PixcelFormatBGR; break;
                case PixelFormat.Format32bppArgb:
                case PixelFormat.Format32bppPArgb:
                    pixelFormat = TJPixelFormat.PixcelFormatBGRA; break;
                case PixelFormat.Format8bppIndexed:
                    pixelFormat = TJPixelFormat.PixcelFormatGRAY; break;
                case PixelFormat.Format32bppRgb:
                    pixelFormat = TJPixelFormat.PixcelFormatBGRX;
                    intPixels = true; break;
                default:
                    throw new ArgumentException("Unsupported Bitmap format");
            }
            var bitmapLock = dstImage.LockBits(new Rectangle(Point.Empty, dstImage.Size), System.Drawing.Imaging.ImageLockMode.ReadWrite, dstImage.PixelFormat);

            IntPtr buf = bitmapLock.Scan0;
            int stride = bitmapLock.Stride;
            if (jpegBuf == null)
                throw new InvalidOperationException();
            DecompressInternal(jpegBuf, jpegBufSize, buf, 0, 0, scaledWidth, stride, scaledHeight,
                       pixelFormat, flags);
            dstImage.UnlockBits(bitmapLock);
        }

        public Bitmap Decompress(int desiredWidth, int desiredHeight,
                                        PixelFormat bufferedImageType, TJFlag flags)
        {
            if (desiredWidth < 0 || desiredHeight < 0 || flags < 0)
                throw new ArgumentException("Invalid argument in decompress()");
            int scaledWidth = GetScaledWidth(desiredWidth, desiredHeight);
            int scaledHeight = GetScaledHeight(desiredWidth, desiredHeight);
            Bitmap img = new Bitmap(scaledWidth, scaledHeight,
                                                  bufferedImageType);
            Decompress(img, flags);
            return img;
        }

        public void Close()
        {
            Dispose();
        }

#if USE_UNSAFE
        [DllImport("turbojpeg.dll", CallingConvention = CallingConvention.Cdecl)]
        private static extern unsafe int tjDecompressToYUV(tjhandle handle, byte* jpegBuf, uint jpegSize, byte* dstBuf,int flags);
        
        [DllImport("turbojpeg.dll", CallingConvention = CallingConvention.Cdecl)]
        private static extern unsafe int tjDecompress2(tjhandle handle,  byte* jpegBuf, uint jpegSize, byte* dstBuf,  int width, int pitch, int height, int pixelFormat, int flags);

        [DllImport("turbojpeg.dll", CallingConvention = CallingConvention.Cdecl)]
        private static extern unsafe int tjDecompressHeader2(tjhandle handle, byte* jpegBuff, uint jpegSize, ref int width, ref int height, ref int jpegSubsamp);

#endif
        [DllImport("turbojpeg.dll", CallingConvention = CallingConvention.Cdecl)]
        private static extern int tjDecompressHeader2(tjhandle handle, IntPtr jpegBuff, uint jpegSize, out int width, out int height, out int jpegSubsamp);

        [DllImport("turbojpeg.dll", CallingConvention = CallingConvention.Cdecl)]
        private static extern int tjDecompressHeader2(tjhandle handle, byte[] jpegBuff, uint jpegSize, out int width, out int height, out int jpegSubsamp);

        [DllImport("turbojpeg.dll", CallingConvention = CallingConvention.Cdecl)]
        private static extern int tjDecompressToYUV(tjhandle handle, IntPtr jpegBuf, uint jpegSize, IntPtr dstBuf, int flags);

        [DllImport("turbojpeg.dll", CallingConvention = CallingConvention.Cdecl)]
        private static extern int tjDecompressToYUV(tjhandle handle, byte[] jpegBuf, uint jpegSize, byte[] dstBuf, int flags);

        [DllImport("turbojpeg.dll", CallingConvention = CallingConvention.Cdecl)]
        private static extern int tjDecompress2(tjhandle handle, byte[] jpegBuf, uint jpegSize, IntPtr dstBuf, int width, int pitch, int height, int pixelFormat, int flags);

        [DllImport("turbojpeg.dll", CallingConvention = CallingConvention.Cdecl)]
        private static extern int tjDecompress2(tjhandle handle, byte[] jpegBuf, uint jpegSize, byte[] dstBuf, int width, int pitch, int height, int pixelFormat, int flags);


        private void DecompressInternal(byte[] srcBuf, int size, byte[] dstBuf, int x, int y, int width, int pitch, int height, TJPixelFormat pixelFormat, TJFlag flags)
        {
            TJ.CheckPixelFormat(pixelFormat, "Invalid argument in decompress()");

            if (srcBuf.Length < size)
                throw new ArgumentException("Source buffer is not large enough");

            int actualPitch = pitch == 0 ? width * TJ.getPixelSize(pixelFormat) : pitch;
            int arraySize = (y + height - 1) * actualPitch + (x + width) * TJ.getPixelSize(pixelFormat);

            if (dstBuf.Length < arraySize)
                throw new Exception("Destination buffer is not large enough");

            var gcHandle = GCHandle.Alloc(dstBuf, GCHandleType.Pinned);
            var ptr = Marshal.UnsafeAddrOfPinnedArrayElement(dstBuf, y * actualPitch + x * TJ.getPixelSize(pixelFormat));

            try
            {
                if (tjDecompress2(handle, srcBuf, (uint)size, dstBuf, width, pitch, height, (int)pixelFormat, (int)flags) == -1)
                    throw new TJException(TJ.tjGetErrorStr());
            }
            catch
            {
                gcHandle.Free();
            }
        }
        private void DecompressInternal(byte[] srcBuf, int size, IntPtr dstBuf, int x, int y, int desiredWidth, int pitch, int desiredHeight, TJPixelFormat pixelFormat, TJFlag flags)
        {
            TJ.CheckPixelFormat(pixelFormat, "Invalid argument in decompress()");

            if (srcBuf.Length < size)
                throw new ArgumentException("Source buffer is not large enough");

            int actualPitch = pitch == 0 ? desiredWidth * TJ.getPixelSize(pixelFormat) : pitch;
            int arraySize = (y + desiredHeight - 1) * actualPitch + (x + desiredWidth) * TJ.getPixelSize(pixelFormat);

            var ptr = new IntPtr(dstBuf.ToInt64() + y * actualPitch + x * TJ.getPixelSize(pixelFormat)); ;

            if (tjDecompress2(handle, srcBuf, (uint)size, ptr, desiredWidth, pitch, desiredHeight, (int)pixelFormat, (int)flags) == -1)
                throw new TJException(TJ.tjGetErrorStr());
        }

        private void DecompressInternal(byte[] srcBuf, int size, int[] dstBuf, int x, int y, int desiredWidth, int stride, int desiredHeight, TJPixelFormat pixelFormat, TJFlag flags)
        {
            TJ.CheckPixelFormat(pixelFormat, "Invalid argument in decompress()");

            if (srcBuf.Length < size)
                throw new ArgumentException("Source buffer is not large enough");

            int actualStride = stride == 0 ? desiredWidth : stride;
            int arraySize = (y + desiredHeight - 1) * actualStride + x + desiredWidth;

            if (dstBuf.Length < arraySize)
                throw new Exception("Destination buffer is not large enough");

            var gcHandle = GCHandle.Alloc(dstBuf, GCHandleType.Pinned);
            var ptr = Marshal.UnsafeAddrOfPinnedArrayElement(dstBuf, (y * actualStride + x));

            try
            {
                if (tjDecompress2(handle, srcBuf, (uint)size, ptr, desiredWidth, stride * sizeof(int), desiredHeight, (int)pixelFormat, (int)flags) == -1)
                    throw new TJException(TJ.tjGetErrorStr());
            }
            catch
            {
                gcHandle.Free();
            }
        }

        private void DecompressToYUV(byte[] srcBuf, int size, byte[] dstBuf, TJFlag flags)
        {
            if (srcBuf.Length < size)
                throw new ArgumentException("Source buffer is not large enough");
            if (dstBuf.Length < TJ.tjBufSizeYUV(jpegWidth, jpegHeight, (int)jpegSubsamp))
                throw new Exception("Destination buffer is not large enough");

            if (tjDecompressToYUV(handle, srcBuf, (uint)size, dstBuf, (int)flags) == -1)
                throw new TJException(TJ.tjGetErrorStr());
        }

        protected tjhandle handle;
        protected byte[] jpegBuf = null;
        protected int jpegBufSize = 0;
        protected int jpegWidth = 0;
        protected int jpegHeight = 0;
        protected TJSamp jpegSubsamp = (TJSamp)(-1);

        private bool disposed;
        protected virtual void Dispose(bool disposing)
        {
            if (!disposed)
            {
                if (disposing)
                {
                    handle.Dispose();
                }
                handle = null;
                disposed = true;
            }
        }
        #region IDisposable メンバー

        public void Dispose()
        {
            Dispose(true);
            GC.SuppressFinalize(this);
        }

        #endregion
    }
}
