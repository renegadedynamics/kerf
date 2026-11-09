// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;

namespace Kerf.Kernel.Foundation.Tests;

/// <summary>The memory the readers' and writers' async forms go through (unit 14G): a stream as a MemoryStream behaves.</summary>
public class MemorySpoolTests
{
    // Sizes either side of the chunks' bounds: 16 KB, 16 + 32 KB, …, the first megabyte chunk, and past it
    private static readonly int[] Sizes = [0, 1, 16 << 10, (16 << 10) + 1, (48 << 10) - 1, 48 << 10, (1 << 20) - (16 << 10), 1 << 20, (3 << 20) + 12345];

    private static byte[] Data(int length)
    {
        var data = new byte[length];
        new Random(length).NextBytes(data);
        return data;
    }

    [Fact]
    public async Task What_is_read_is_the_source_up_to_the_limit_and_one_more()
    {
        foreach (int size in Sizes)
        {
            byte[] data = Data(size);
            using (MemorySpool all = await MemorySpool.ReadFromAsync(new MemoryStream(data), long.MaxValue, CancellationToken.None))
            {
                Assert.Equal(size, all.Length);
                Assert.Equal(0, all.Position);
                var back = new MemoryStream();
                all.CopyTo(back, 7777);
                Assert.Equal(data, back.ToArray());
            }

            if (size > 2)
            {
                using MemorySpool limited = await MemorySpool.ReadFromAsync(new MemoryStream(data), size / 2, CancellationToken.None);
                Assert.Equal(size / 2 + 1, limited.Length);
                var back = new MemoryStream();
                limited.CopyTo(back);
                Assert.Equal(data[..(size / 2 + 1)], back.ToArray());
            }
        }

        using var cancelled = new CancellationTokenSource();
        cancelled.Cancel();
        await Assert.ThrowsAnyAsync<OperationCanceledException>(() => MemorySpool.ReadFromAsync(new MemoryStream(Data(10)), 100, cancelled.Token));
    }

    [Fact]
    public async Task What_is_written_is_written_out_whole_and_reads_back_as_a_MemoryStream_would()
    {
        foreach (int size in Sizes)
        {
            byte[] data = Data(size);
            using var spool = new MemorySpool();
            var reference = new MemoryStream();
            // Written in uneven pieces, with seeks back and over-writes, as a writer patching a header does
            for (int at = 0; at < size;)
            {
                int count = Math.Min(size - at, 1 + (at % 70001));
                spool.Write(data, at, count);
                reference.Write(data, at, count);
                at += count;
            }

            if (size > 8)
            {
                spool.Position = 3;
                reference.Position = 3;
                spool.Write([1, 2, 3, 4]);
                reference.Write([1, 2, 3, 4]);
                spool.Seek(-2, SeekOrigin.End);
                reference.Seek(-2, SeekOrigin.End);
                spool.WriteByte(9);
                reference.WriteByte(9);
            }

            // Past the end: zeros between
            spool.Seek(5, SeekOrigin.End);
            reference.Seek(5, SeekOrigin.End);
            spool.WriteByte(7);
            reference.WriteByte(7);
            Assert.Equal(reference.Length, spool.Length);

            var written = new MemoryStream();
            await spool.WriteToAsync(written, CancellationToken.None);
            Assert.Equal(reference.ToArray(), written.ToArray());

            spool.Position = 0;
            var readBack = new byte[reference.Length + 10];
            int read = 0, n;
            while ((n = spool.Read(readBack, read, Math.Min(4099, readBack.Length - read))) > 0)
            {
                read += n;
            }

            Assert.Equal(reference.Length, read);
            Assert.Equal(reference.ToArray(), readBack[..read]);
            Assert.Equal(-1, spool.ReadByte());
        }
    }

    [Fact]
    public void SetLength_cuts_and_extends_with_zeros()
    {
        using var spool = new MemorySpool();
        spool.Write(Data(100_000));
        spool.SetLength(10);
        spool.SetLength(50_000);
        spool.Position = 0;
        var back = new byte[50_000];
        Assert.Equal(50_000, spool.Read(back));
        Assert.Equal(Data(100_000)[..10], back[..10]);
        Assert.All(back[10..], b => Assert.Equal(0, b));
    }
}
