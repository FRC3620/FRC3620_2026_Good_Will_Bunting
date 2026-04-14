#!/usr/bin/env python3
"""MIDI Analyzer for CTRE Orchestra Track Assignment"""

import mido
from mido import MidiFile

def analyze_midi_for_orchestra(midi_file):
    mid = MidiFile(midi_file)
    
    print(f"\n=== MIDI Analysis: {midi_file} ===\n")
    print(f"Tracks: {len(mid.tracks)}")
    print(f"Ticks per beat: {mid.ticks_per_beat}\n")
    
    for i, track in enumerate(mid.tracks):
        note_count = 0
        note_range = set()
        max_polyphony = 0
        current_poly = 0
        
        for msg in track:
            if not msg.is_meta and msg.type == 'note_on' and msg.velocity > 0:
                note_count += 1
                note_range.add(msg.note)
                current_poly += 1
                max_polyphony = max(max_polyphony, current_poly)
            elif not msg.is_meta and (msg.type == 'note_off' or (msg.type == 'note_on' and msg.velocity == 0)):
                current_poly = max(0, current_poly - 1)
        
        track_name = track.name if track.name else f"Track {i}"
        melody_score = "⭐" if (len(note_range) > 24 and max_polyphony <= 4) else ""
        
        print(f"Track {i}: '{track_name}' {melody_score}")
        print(f"  Notes: {note_count}")
        print(f"  Note range: {min(note_range) if note_range else 'N/A'} to {max(note_range) if note_range else 'N/A'} ({len(note_range)} unique)")
        print(f"  Max polyphony: {max_polyphony}")
        print()

if __name__ == "__main__":
    import sys
    if len(sys.argv) < 2:
        print("Usage: python midi_analyzer.py <file.mid>")
    else:
        analyze_midi_for_orchestra(sys.argv[1])