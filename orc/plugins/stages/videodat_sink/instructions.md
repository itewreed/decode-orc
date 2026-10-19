# VideoDat Sink

Decodes Channel VideoDat, the data service Pro 7 broadcast in the vertical blanking interval from 1993 to 1995, and writes the files it carried, together with a report of its other services.

## When to use

Add this sink to a PAL recording of Pro 7 from the VideoDat years when you want the software, pictures and text files that were transmitted with the programme. It accepts:

- frames from **TBC Source** reading a decoder's `.tbc`; line sync is then used as the timing reference of each data line;
- frames from **VBI Source** with the preset `bt8x8 card dump, 8-bit (VideoDat)` (a raw `/dev/vbi` capture) or `.tbc VBI crop, 16-bit (VideoDat)` (the VBI lines of a `.tbc`, cut out on their own).

## What it does

VideoDat sends ten bytes per VBI line at 2 Mbit/s, each byte framed by a start and a stop bit. The stage:

1. **Finds the data lines.** The first 40 lines of each field are searched for lines whose start and stop bits all read correctly, for up to 30 seconds from the start of the recording. Lines framed in at least 30 % of the fields the best line was are taken; the choice is reported as PAL line numbers.
2. **Times each line.** On input with line sync, the distance from the end of sync to the first start bit is measured on perfectly framed lines and used for every line, so a line whose start edge is damaged is still read. Without sync, the start edge itself is used, and with **Track Data Lines** a group of data lines that moves within the field is followed.
3. **Slices the bits.** With **Train Equaliser**, a nine-tap linear equaliser and a decision threshold are learnt from the recording's own error-free lines, beginning after the line search. Every data bit keeps how far it was from the threshold.
4. **Recovers the packets.** Packets are found by their sync word, descrambled and checked against their checksum. Every packet is sent at least twice: damaged packets are repaired from their copies, bit by bit from the summed soft values, by combining bytes, or by joining the head of one copy to the tail of another.
5. **Reassembles the files.** File headers give name, size and date; data packets follow in counter order. Damaged headers are recovered where the packet counter and the file length confirm them, files sent more than once are combined, and blocks received in different versions are decided by majority.

## Output

The report is written to **Report File**. The files go into a folder next to it with the same name, without the extension (`tape1.txt` → `tape1/`). The layout and names are those of the reference decoder, channel-videodat-decoder, so its tools for further analysis read them unchanged:

| Path | Content |
|------|---------|
| `strom_XX/NAME.EXT` | A file from data stream XX, with its transmitted date. A second, different file of the same name gets a number (`NAME_2.EXT`). |
| `strom_XX/NAME.EXT.unvollstaendig` | A file not received completely; missing blocks are zeros. |
| `strom_XX/ohne_dateikopf_N.bin.unvollstaendig` | Data blocks whose file header was not received, from counter N on. |
| `dienste/dienste.txt` | Clock, transmission schedule, active streams, bit lists and encrypted services. |
| `dienste/bitliste_XX.bin` | Bit list XX, one bit per decoder or subscriber number. |
| `dienste/dienst_XX_typYY.bin` | The distinct packets of an encrypted or unexplained service, each as a two-byte little-endian length and its payload. |
| `andere_dienste/strom_XX_typ_YY.bin` | With **Save Other Packets**: every packet of a service other than file transfer. |
| `videodat_bytes.bin` | With **Save Byte Stream**: the recovered byte stream, ten bytes per data line. |

Purchased software (files beginning `CHVDFILE`) was encrypted for the subscriber who paid for it; the report names the file it holds, but its content cannot be read without the receiving software and the account disk. Decrypting the account services is not part of this stage.

## Parameters

### output_path (file path)
Where the report is written; the files go into the folder of the same name. Required.

### write_partial (bool)
Also write incomplete files and data without a file header. Default: on.

### analyse_services (bool)
Analyse the services besides file transfer and write the `dienste` folder. Default: on.

### dump_other (bool)
Save the packets of every other service raw. Default: off.

### save_byte_stream (bool)
Save the recovered byte stream. Default: off.

### use_sync (bool)
Time each line from the end of its line sync where the input carries it. Default: on.

### use_equaliser (bool)
Learn an equaliser and decision threshold from the recording. Default: on.

### track_lines (bool)
Follow data lines that move within the field, on input without sync. Default: on.

## Notes

The stage reads PAL input only. Do not blank the VBI lines (for example with Mask Line) before this stage. A recording without VideoDat produces a report saying no lines were found and no files.

## Status Indicator

The coloured dot in the top-right corner of the node shows its configuration status.

| Colour | Meaning |
|--------|---------|
| Green | Fully configured and ready to run. All required parameters are set. |
| Yellow | Partially configured. The stage can run but will use default or reduced behaviour — for example, pass-through mode or console-only output. Review the parameters for optional settings. |
| Red | Not configured. One or more required parameters are missing and the stage cannot run. |

Parameters can be set via **Edit Parameters...** in the node context menu. Some stages also provide interactive stage tools (listed under **Tools** above) that set parameters directly from within the tool.
