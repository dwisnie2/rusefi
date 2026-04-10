/*
 * @file trigger_rover.cpp
 *
 * @date Dec 27, 2015
 * @author PhilTheGeek
 * @author Andrey Belomutskiy, (c) 2012-2020
 */

#include "pch.h"

#include "trigger_rover.h"

/**
 * https://en.wikipedia.org/wiki/Rover_K-series_engine
 */
void initializeRoverK(TriggerWaveform *s) {
	s->initialize(FOUR_STROKE_CRANK_SENSOR, SyncEdge::RiseOnly);

	float tooth = 20;

	s->setTriggerSynchronizationGap(2.0);
	// wow that's odd
	s->setSecondTriggerSynchronizationGap2(0.0001, 100000);
	s->setThirdTriggerSynchronizationGap(2);

	float base = 0;

	for (int i = 0; i < 2; i++) {
		s->addEvent720(base + tooth / 2, TriggerValue::RISE);
		s->addEvent720(base + tooth, TriggerValue::FALL);
		base += tooth;
	}
	base += tooth;
	for (int i = 0; i < 2; i++) {
		s->addEvent720(base + tooth / 2, TriggerValue::RISE);
		s->addEvent720(base + tooth, TriggerValue::FALL);
		base += tooth;
	}
	base += tooth;
	for (int i = 0; i < 14; i++) {
		s->addEvent720(base + tooth / 2, TriggerValue::RISE);
		s->addEvent720(base + tooth, TriggerValue::FALL);
		base += tooth;
	}
	base += tooth;
	for (int i = 0; i < 3; i++) {
		s->addEvent720(base + tooth / 2, TriggerValue::RISE);
		s->addEvent720(base + tooth, TriggerValue::FALL);
		base += tooth;
	}
	base += tooth;
	for (int i = 0; i < 11; i++) {
		s->addEvent720(base + tooth / 2, TriggerValue::RISE);
		s->addEvent720(base + tooth, TriggerValue::FALL);
		base += tooth;
	}

}

/**
 * MG Rover MEMS3 Common Pattern 2 — 36-slot crank, 4 single missing teeth.
 * Tooth groups between gaps: 2, 14, 3, 13
 * Missing teeth at 30°, 60°, 210°, 250° ATDC.
 *
 * Pattern starts at TDC (0° ATDC) so that globalTriggerAngleOffset = 0.
 * Physical layout from TDC:
 *   3 teeth (tail of group-13) — gap_D @30° — 2 teeth — gap_A @60°
 *   — 14 teeth — gap_B @210° — 3 teeth — gap_C @250° — 10 teeth (head of group-13)
 *
 * Sync on gap_A — unique ratio sequence [2.0, 0.5, 2.0]:
 *   dur[0]=40 (gap_A), dur[1]=20 (tooth in group-2), dur[2]=40 (gap_D)
 *   gap[0]=40/20=2.0  gap[1]=20/40=0.5  gap[2]=40/20=2.0
 * Other gaps all produce different sequences.
 */
void initializeRoverK16(TriggerWaveform *s) {
	s->initialize(FOUR_STROKE_CRANK_SENSOR, SyncEdge::RiseOnly);

	float tooth = 20;
	float base = 0;

	// 3 teeth — tail of group-13 (0°, 10°, 20° ATDC)
	for (int i = 0; i < 3; i++) {
		s->addEvent720(base + tooth / 2, TriggerValue::RISE);
		s->addEvent720(base + tooth, TriggerValue::FALL);
		base += tooth;
	}
	base += tooth;  // gap_D (30° ATDC)

	// group of 2 (40°, 50° ATDC)
	for (int i = 0; i < 2; i++) {
		s->addEvent720(base + tooth / 2, TriggerValue::RISE);
		s->addEvent720(base + tooth, TriggerValue::FALL);
		base += tooth;
	}
	base += tooth;  // gap_A (60° ATDC) — sync point

	// group of 14 (70°–200° ATDC)
	for (int i = 0; i < 14; i++) {
		s->addEvent720(base + tooth / 2, TriggerValue::RISE);
		s->addEvent720(base + tooth, TriggerValue::FALL);
		base += tooth;
	}
	base += tooth;  // gap_B (210° ATDC)

	// group of 3 (220°, 230°, 240° ATDC)
	for (int i = 0; i < 3; i++) {
		s->addEvent720(base + tooth / 2, TriggerValue::RISE);
		s->addEvent720(base + tooth, TriggerValue::FALL);
		base += tooth;
	}
	base += tooth;  // gap_C (250° ATDC)

	// 10 teeth — head of group-13 (260°–350° ATDC)
	for (int i = 0; i < 10; i++) {
		s->addEvent720(base + tooth / 2, TriggerValue::RISE);
		s->addEvent720(base + tooth, TriggerValue::FALL);
		base += tooth;
	}
	// base = 720 ✓

	s->setTriggerSynchronizationGap3(/*gapIndex*/0, 1.5f, 2.5f);   // ~2.0
	s->setTriggerSynchronizationGap3(/*gapIndex*/1, 0.35f, 0.65f); // ~0.5
	s->setTriggerSynchronizationGap3(/*gapIndex*/2, 1.5f, 2.5f);   // ~2.0
}
