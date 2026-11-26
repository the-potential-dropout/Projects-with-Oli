-- Hexapod Mk1 telemetry screen for EdgeTX black-and-white radios (Boxer, Pocket, Zorro).
-- Copy to /SCRIPTS/TELEMETRY/hexa.lua, then pick it on the model's Display page.

local function v(n) local x = getValue(n); return x or 0 end

local function run(event)
  lcd.clear()
  local volt, amp, pct = v("RxBt"), v("Curr"), v("Bat%")
  local roll, pitch, lq = v("Roll"), v("Ptch"), v("RQly")
  local fm = getValue("FM")
  lcd.drawText(1, 1, "HEXAPOD MK1", INVERS)
  if type(fm) == "string" then lcd.drawText(70, 1, fm, (fm:find("!") and BLINK or 0)) end
  lcd.drawText(1, 14, string.format("%.2fV", volt), DBLSIZE + ((volt > 0 and volt < 10.5) and BLINK or 0))
  lcd.drawText(70, 14, string.format("%.1fA", amp), MIDSIZE)
  lcd.drawGauge(1, 34, 100, 6, pct, 100)
  lcd.drawText(1, 44, string.format("R%5.1f P%5.1f", math.deg(roll), math.deg(pitch)), SMLSIZE)
  lcd.drawText(1, 54, string.format("LQ %d%%", lq), SMLSIZE + ((lq < 50) and BLINK or 0))
  return 0
end

return { run = run }
