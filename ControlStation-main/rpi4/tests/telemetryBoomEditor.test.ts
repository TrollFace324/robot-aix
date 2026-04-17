import test from "node:test";
import assert from "node:assert/strict";

import {
  appendDigitToTelemetryMmDraft,
  backspaceTelemetryMmDraft,
  buildTelemetryBoomSlotOptions,
  isTelemetryBoomMmInRange,
  normalizeTelemetryMmDraft,
  parseTelemetryBoomSlots,
  shouldClearTelemetryBoomSlots,
  shouldLoadTelemetryBoomSlots,
  telemetryBoomMaxMm,
  telemetryBoomMinMm,
  telemetryBoomSlotCount,
} from "../src/telemetryBoomEditor.ts";

test("builds boom slot options for positions 0 through 10", () => {
  const options = buildTelemetryBoomSlotOptions();

  assert.equal(options.length, telemetryBoomSlotCount);
  assert.deepEqual(options[0], { slot: 0, label: "Позиция 0" });
  assert.deepEqual(options[10], { slot: 10, label: "Позиция 10" });
});

test("appends keypad digits to telemetry mm draft", () => {
  assert.equal(appendDigitToTelemetryMmDraft("0", "6"), "6");
  assert.equal(appendDigitToTelemetryMmDraft("6", "6"), "66");
  assert.equal(appendDigitToTelemetryMmDraft("066", "0"), "660");
  assert.equal(appendDigitToTelemetryMmDraft("1234", "5"), "1234");
});

test("backs up keypad draft and keeps zero floor", () => {
  assert.equal(backspaceTelemetryMmDraft("660"), "66");
  assert.equal(backspaceTelemetryMmDraft("6"), "0");
  assert.equal(backspaceTelemetryMmDraft("0"), "0");
});

test("normalizes telemetry mm draft before save", () => {
  assert.equal(normalizeTelemetryMmDraft("0066"), "66");
  assert.equal(normalizeTelemetryMmDraft(""), "0");
  assert.equal(normalizeTelemetryMmDraft("12ab34"), "1234");
});

test("accepts telemetry mm only inside configured range", () => {
  assert.equal(isTelemetryBoomMmInRange(telemetryBoomMinMm), true);
  assert.equal(isTelemetryBoomMmInRange(telemetryBoomMaxMm), true);
  assert.equal(isTelemetryBoomMmInRange(2001), false);
  assert.equal(isTelemetryBoomMmInRange(-1), false);
  assert.equal(isTelemetryBoomMmInRange(12.5), false);
});

test("parses boom slots payload from master api", () => {
  assert.deepEqual(
    parseTelemetryBoomSlots([110, 99, 88, 77, 66, 55, 44, 33, 22, 11, 0]),
    [110, 99, 88, 77, 66, 55, 44, 33, 22, 11, 0],
  );
  assert.equal(parseTelemetryBoomSlots([110, 99]), null);
  assert.equal(parseTelemetryBoomSlots(["110"]), null);
});

test("reloads boom slots only when telemetry reconnects with fresh summary", () => {
  assert.equal(
    shouldLoadTelemetryBoomSlots({
      isTelemetryTabActive: true,
      isBoomEditorSelected: true,
      isSystemSummaryStale: false,
    }),
    true,
  );
  assert.equal(
    shouldLoadTelemetryBoomSlots({
      isTelemetryTabActive: true,
      isBoomEditorSelected: true,
      isSystemSummaryStale: true,
    }),
    false,
  );
  assert.equal(
    shouldLoadTelemetryBoomSlots({
      isTelemetryTabActive: false,
      isBoomEditorSelected: true,
      isSystemSummaryStale: false,
    }),
    false,
  );
});

test("clears cached boom slots while robot summary is stale", () => {
  assert.equal(
    shouldClearTelemetryBoomSlots({
      isTelemetryTabActive: true,
      isBoomEditorSelected: true,
      isSystemSummaryStale: true,
    }),
    true,
  );
  assert.equal(
    shouldClearTelemetryBoomSlots({
      isTelemetryTabActive: true,
      isBoomEditorSelected: true,
      isSystemSummaryStale: false,
    }),
    false,
  );
});
