-- Hexapod Mk1 telemetry widget for EdgeTX colour radios (RadioMaster TX16S MK3 etc.).
-- Copy the HexaMk1 folder to /WIDGETS on the radio's SD card, then add the widget to a
-- full-screen or 1/2-screen zone.  It reads the CRSF telemetry the robot sends
-- (firmware/hexapod_controller: battery 0x08, attitude 0x1E, flight mode 0x21).
-- Run "Discover new sensors" once on the Telemetry page so RxBt / Curr / Bat% / Ptch /
-- Roll / FM / RQly exist.

local name = "HexaMk1"
local options = {
  { "Cells", VALUE, 3, 1, 6 },
  { "Warn", VALUE, 34, 30, 40 },   -- warn below 3.4 V per cell (x10)
}

local function create(zone, opts)
  return { zone = zone, options = opts, minV = 99 }
end

local function update(w, opts)
  w.options = opts
end

local function val(n, d)
  local v = getValue(n)
  if v == nil or v == 0 and d ~= nil then return d end
  return v
end

local function bar(x, y, w, h, frac, color)
  lcd.drawRectangle(x, y, w, h, COLOR_THEME_SECONDARY1)
  frac = math.max(0, math.min(1, frac))
  lcd.drawFilledRectangle(x + 1, y + 1, math.floor((w - 2) * frac), h - 2, color)
end

-- artificial horizon: roll and pitch of the body
local function horizon(cx, cy, r, roll, pitch)
  local s, c = math.sin(roll), math.cos(roll)
  local off = pitch / math.rad(30) * r
  local x1, y1 = cx - c * r, cy + s * r + off
  local x2, y2 = cx + c * r, cy - s * r + off
  lcd.drawCircle(cx, cy, r, COLOR_THEME_SECONDARY1)
  lcd.drawLine(x1, y1, x2, y2, SOLID, COLOR_THEME_WARNING)
  lcd.drawLine(cx - 6, cy, cx + 6, cy, SOLID, COLOR_THEME_PRIMARY1)
end

local function refresh(w, event, touch)
  local z = w.zone
  local x, y = z.x + 6, z.y + 4
  local v = val("RxBt", 0)
  local a = val("Curr", 0)
  local pct = val("Bat%", 0)
  local roll = val("Roll", 0)        -- radians (CRSF attitude)
  local pitch = val("Ptch", 0)
  local fm = getValue("FM")
  local lq = val("RQly", 0)
  local cells = w.options.Cells
  local warn = w.options.Warn / 10 * cells
  local col = (v > 0 and v < warn) and COLOR_THEME_WARNING or COLOR_THEME_PRIMARY1

  lcd.drawText(x, y, "Hexapod Mk1", MIDSIZE + COLOR_THEME_PRIMARY1)
  if type(fm) == "string" then lcd.drawText(x + 170, y, fm, MIDSIZE + (fm:find("!") and BLINK or 0)) end
  y = y + 32
  lcd.drawText(x, y, string.format("%.2f V  %.1f A", v, a), DBLSIZE + col)
  y = y + 44
  bar(x, y, 200, 14, pct / 100, col)
  lcd.drawText(x + 206, y - 2, string.format("%d%%", pct), 0)
  y = y + 24
  lcd.drawText(x, y, string.format("roll %5.1f  pitch %5.1f", math.deg(roll), math.deg(pitch)), 0)
  y = y + 20
  lcd.drawText(x, y, string.format("link %d%%", lq), (lq < 50) and (BLINK + COLOR_THEME_WARNING) or 0)
  if z.w > 300 then horizon(z.x + z.w - 70, z.y + 80, 55, roll, pitch) end
  if v > 0 and v < warn and getTime() % 200 < 5 then playTone(1200, 120, 0) end
end

return { name = name, options = options, create = create, update = update, refresh = refresh }
