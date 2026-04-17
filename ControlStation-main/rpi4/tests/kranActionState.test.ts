import test from "node:test";
import assert from "node:assert/strict";

import {
  shouldResetKranActionToHomeOnFailure,
  shouldResetKranActionToHomeOnSuccess,
  shouldResetKranActionToHomeOnTimeout,
} from "../src/kranActionState.ts";

test("resets kran action to home after successful home and throw requests", () => {
  assert.equal(shouldResetKranActionToHomeOnSuccess("11_0"), true);
  assert.equal(shouldResetKranActionToHomeOnSuccess("12_4"), true);
  assert.equal(shouldResetKranActionToHomeOnSuccess("6"), false);
});

test("resets kran action to home for any failed kran request", () => {
  assert.equal(shouldResetKranActionToHomeOnFailure("11_0"), true);
  assert.equal(shouldResetKranActionToHomeOnFailure("12_7"), true);
  assert.equal(shouldResetKranActionToHomeOnFailure("6"), false);
});

test("resets kran action to home when kran request times out", () => {
  assert.equal(
    shouldResetKranActionToHomeOnTimeout({
      requestId: "12_3",
      statusCode: 504,
      errorMessage: "Second slave response timeout",
    }),
    true,
  );

  assert.equal(
    shouldResetKranActionToHomeOnTimeout({
      requestId: "11_0",
      statusCode: 200,
      errorMessage: "Slave response timeout",
    }),
    true,
  );
});

test("does not reset kran action to home for non-timeout errors", () => {
  assert.equal(
    shouldResetKranActionToHomeOnTimeout({
      requestId: "12_5",
      statusCode: 409,
      errorMessage: "BUSY",
    }),
    false,
  );

  assert.equal(
    shouldResetKranActionToHomeOnTimeout({
      requestId: "6",
      statusCode: 504,
      errorMessage: "Slave response timeout",
    }),
    false,
  );
});
