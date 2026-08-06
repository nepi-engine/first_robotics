# Editing Guide — nepi_app_controls_sandbox

How to change what the Controls and Data panels show, add and remove items, and
wire each one back to the ROS node.

Written for a developer who has never written JavaScript. Every instruction here
points at a real line in this app. Nothing is hypothetical.

Read the `README.md` in this directory first if you have not. It covers what the
app is for. This guide covers how to change it.

---

## 1. What these files are

Four JavaScript files draw the app's page in the RUI. One Python file owns
everything the page displays. Here is the map.

```
nepi_app_controls_sandbox/
├── rui/
│   ├── NepiAppControlsSandbox.js           # the page: lays out the three boxes
│   ├── NepiAppControlsSandbox-Controls.js  # the CONTROLS SANDBOX box
│   ├── NepiAppControlsSandbox-Data.js      # the DATA SANDBOX box
│   └── NepiAppControlsSandbox-Settings.js  # the CONTROLS SETTINGS box (develop mode only)
├── scripts/
│   └── controls_sandbox_app_node.py        # the ROS node: declares every control and datum
├── msg/
│   └── ControlsSandboxStatus.msg           # this app's own small status message
└── params/
    └── controls_sandbox_app_params.yaml    # app metadata + the RUI file registration list
```

### Which file do I edit?

| I want to… | Edit this |
| --- | --- |
| Add a control, or change its label, default, bounds, options | `scripts/controls_sandbox_app_node.py`, `createControlsInitDict()` |
| Add a datum, or change its label | `scripts/controls_sandbox_app_node.py`, `createDataInitDict()` |
| Change what a datum's *value* is | `scripts/controls_sandbox_app_node.py`, `dataUpdaterCb()` |
| Change how **every control of one type** is drawn | `rui/NepiAppControlsSandbox-Controls.js`, `renderControl()` |
| Change how **every datum of one type** is drawn | `rui/NepiAppControlsSandbox-Data.js`, `renderDatum()` |
| Change the box titles or the page layout | `rui/NepiAppControlsSandbox.js`, `render()` |
| Register a new `.js` file | `params/controls_sandbox_app_params.yaml`, `RUI_DICT.rui_files` |

### The one thing to understand before you edit anything

**There is no JavaScript block for "Demo Menu". There is no JavaScript block for
"Demo Int".** The Controls box does not know the names of this app's controls.

What it has is one block per *control type*. `renderControl()` in
`NepiAppControlsSandbox-Controls.js:249` receives a control's name, its declared
type, and its data, and picks a widget by looking at the type:

```jsx
    // MENU -- drop-down of string options; the control's value is the *index*
    // of the selected option. Sends the new index as an Int.
    if (type === "Menu") {
```
*(`NepiAppControlsSandbox-Controls.js:257-259` — the first of ten type branches.)*

The list of controls arrives at runtime, over ROS, in a status message. It is
built in Python. So:

> **Adding, removing or relabelling a control is a Python edit.
> Changing how a whole class of controls looks is a JavaScript edit.**

Most of the changes you will want to make are Python edits, and the JavaScript
never has to be touched. Section 3 covers both, and says which is which for
every task.

The Data box works exactly the same way, with `renderDatum()` in
`NepiAppControlsSandbox-Data.js:139`.

### `Nepi_IF_Apps.js` and `NepiApps.js` are generated. Never edit them.

You may see these two files in `nepi_rui` and think an app has to be registered
there. It does not. `build_nepi_rui.sh` rewrites lines 27 and 31 of
`Nepi_IF_Apps.js` from every app's `RUI_DICT` block. Any edit you make by hand
is overwritten on the next build.

The only registration you own is this, in
`params/controls_sandbox_app_params.yaml`:

```yaml
RUI_DICT:
  rui_files:
  - NepiAppControlsSandbox.js
  - NepiAppControlsSandbox-Controls.js
  - NepiAppControlsSandbox-Data.js
  - NepiAppControlsSandbox-Settings.js
  rui_main_file: NepiAppControlsSandbox.js
  rui_main_class: NepiAppControlsSandbox
```

Add a filename there when you add a `.js` file, and nowhere else.

---

## 2. The round trip

Follow one control all the way around. This is the whole mechanism; once you
have it, everything else in this guide is detail.

**Demo Bool, from the operator's click back to the screen:**

1. **The operator clicks the toggle.** The `onClick` in the `Bool` branch runs:

   ```jsx
        <AsyncToggle
          checked={checked}
          onClick={() => sendUpdateBoolMsg(namespace + "/set_bool_control_value", name, !checked)}
        />
   ```
   *(`NepiAppControlsSandbox-Controls.js:341-344`)*

2. **A ROS message goes out.** `sendUpdateBoolMsg` publishes a
   `nepi_interfaces/UpdateBool` on
   `/nepi/<device>/app_controls_sandbox/controls/set_bool_control_value`. The
   message carries `name: "demo_bool"` and `value: false`. The control's *name*
   is how the node knows which control you meant — it is a field in the
   message, not part of the topic.

3. **The node's interface receives it.** `ControlsIF._setValueCb` in
   `nepi_api/system_if.py` reads `msg.name` and `msg.value` and calls
   `set_control_value('demo_bool', False)`. That writes the new value into the
   node's `controls_dict`, saves it to the ROS param server so it survives a
   restart, and calls this app's `controlsUpdatedCb`:

   ```python
     def controlsUpdatedCb(self, control_name):
       # Called by ControlsIF after a control value/display change is applied.
       value = None
       if self.controls_if is not None:
         value = self.controls_if.get_control_value(control_name)
   ```
   *(`controls_sandbox_app_node.py:310-314`)*

   **This callback is where your app does its actual work.** Right now it only
   logs. Whatever your control is supposed to *do*, it does from here.

4. **The node republishes its status.** `ControlsIF` publishes a
   `nepi_interfaces/ControlsStatus` on `.../controls/status` once per second.
   That message carries three parallel arrays: `controls_name_list`,
   `controls_type_list`, `controls_msg_list`.

5. **The RUI receives it and redraws.** `statusListener()` stores the message,
   React re-renders, and the toggle's `checked` now reads the confirmed backend
   value.

**Data goes the other way and is simpler.** Nothing in the Data box ever
publishes. `dataUpdaterCb()` in the node writes values at 1 Hz, each write
publishes a `nepi_interfaces/DataStatus` on `.../data/status`, and the Data box
displays whatever arrives. The node is the only writer.

**Two namespaces, both derived, never typed by hand.** The page computes them in
`NepiAppControlsSandbox.js`:

```jsx
  getControlsNamespace() {
    // Prefer the namespace advertised by the app status; fall back to the
    // conventional <app>/controls path.
    if (this.state.status_msg != null && this.state.status_msg.controls_namespace) {
      return this.state.status_msg.controls_namespace
    }
```
*(`NepiAppControlsSandbox.js:61-66`)*

The node reports those namespaces in its own `ControlsSandboxStatus` message, so
if you rename the interface in Python the RUI follows automatically.

---

## 3. JavaScript you need, and nothing more

Seven concepts. Every one of them appears in this app. Nothing else in the
language is required to do the edits in this guide.

### 3.1 A component is a class with a `render()` method

A component is one box on the screen. In this app each is a class that extends
`Component` and has a `render()` method returning what to draw.

```jsx
class NepiAppControlsSandboxData extends Component {
  constructor(props) {
    super(props)
```
*(`NepiAppControlsSandbox-Data.js:59-61` — draws the DATA SANDBOX box.)*

`render()` is called again every time the data changes. You never call it
yourself.

### 3.2 JSX tags look like HTML because they are shorthand for building screen elements

The angle-bracket blocks inside `render()` are not strings and not HTML. They
are JavaScript, in a syntax that looks like HTML so that nested layout reads the
way it looks on screen.

```jsx
        <Label title={display_name} key={name}>
          <Input disabled title={description} style={{ width: "100%" }} value={value} />
        </Label>
```
*(`NepiAppControlsSandbox-Data.js:179-181` — one read-only value row: a caption on
the left, a greyed-out box on the right.)*

`<Label>` and `<Input>` are components imported at the top of the file, from
`nepi_rui`. Capitalised tags are components; lowercase tags like `<div>` are
plain HTML elements.

### 3.3 Props are the values you pass into a tag

Everything written as `name={value}` inside a tag is a prop. Props are how a
parent configures a child.

```jsx
                  <NepiAppControlsSandboxControls
                    key={controlsNamespace}
                    namespace={controlsNamespace}
                    make_section={false}
                  />
```
*(`NepiAppControlsSandbox.js:151-155` — the page handing the Controls box the ROS
namespace to subscribe to.)*

Inside the child, those arrive as `this.props.namespace` and
`this.props.make_section`.

### 3.4 Curly braces mean "this is a value, not literal text"

`title="Hidden"` passes the five letters *Hidden*. `title={display_name}` passes
the value of the variable `display_name`.

```jsx
        <Label title={display_name} key={name}>
          <BooleanIndicator title={description} value={(datum_msg.value_bool === true)} />
```
*(`NepiAppControlsSandbox-Data.js:150-151` — a bool readout. Both the caption and
the on/off state come from the ROS message.)*

Braces can hold any expression. `{(datum_msg.value_bool === true)}` evaluates a
comparison and passes the true/false result.

Doubled braces, as in `style={{ width: "100%" }}`, are braces-around-an-object:
the outer pair says "a value follows", the inner pair is the object itself.

### 3.5 `this.state` is what this box knows; `this.props` is what it was handed

```jsx
    const make_section = (this.props.make_section !== undefined) ? this.props.make_section : true
    const status_msg = this.state.status_msg
```
*(`NepiAppControlsSandbox-Controls.js:424-425`)*

`this.props` comes from the parent and never changes on its own.
`this.state` is this box's own memory — here, the most recent ROS status
message. You change state only by calling `this.setState({ ... })`, which
triggers a re-render. Assigning to `this.state.x` directly does nothing visible.

The `a ? b : c` form on that first line reads: *if a, use b, otherwise use c.*

### 3.6 An arrow function is a piece of work to run later

`() => something` means "when this happens, do `something`". It is not run when
the page draws — it is stored, and run on the click.

```jsx
            <Button onClick={() => sendUpdateStringMsg(namespace + "/set_trigger_control_value", name, "")}>{"Trigger"}</Button>
```
*(`NepiAppControlsSandbox-Controls.js:329` — the Trigger button.)*

Leave off the `() =>` and the send fires once while the page is drawing, and
never again on any click. This is the single most common JSX mistake.

### 3.7 `.map()` turns a list of values into a list of tags

To draw one thing per item in a list, `.map()` over the list and return a tag
for each.

```jsx
            {options.map((opt, i) => <Option key={name + '_' + i} value={i}>{opt}</Option>)}
```
*(`NepiAppControlsSandbox-Controls.js:269` — the entries in a Menu dropdown. If the
node declares `['Off', 'Low', 'High']`, this produces three `<Option>` tags.)*

`opt` is each item, `i` is its position starting at 0. React requires a `key`
prop on each item so it can tell them apart between redraws; the pattern in this
app is `key={name + '_' + i}`.

The same trick draws the whole Controls list:

```jsx
            {names.map((name, i) => {
              const control_msg = msgs[i]
              if (control_msg == null) { return null }
```
*(`NepiAppControlsSandbox-Controls.js:458-460`)*

`return null` means "draw nothing for this one" — that is how hidden controls
are skipped.

---

## 4. Rules that will bite you

**Every tag must be closed.** `<Label>` … `</Label>`, or self-closed as
`<Input />` with the slash. A tag left open takes the rest of the file with it.

**One returned block, one outer tag.** A `return (` in JSX may contain exactly
one top-level tag. Two side by side is a syntax error. When you need two, wrap
them — this app uses `<React.Fragment>`, which wraps without adding anything to
the page:

```jsx
    const body = (
      <React.Fragment>
        {show_data_toggle}
        {data_body}
      </React.Fragment>
    )
```
*(`NepiAppControlsSandbox-Data.js:262-267`)*

**Commas between props are wrong; commas between object entries are required.**
Inside a tag, props are separated by whitespace: `<Input disabled value={v} />`.
Inside an object, entries are separated by commas: `{ width: "100%", float:
"left" }`. Mixing the two up is the second most common mistake.

**Strings and numbers are not interchangeable.** `min={0}` passes the number
zero. `min={"0"}` passes the character "0", and arithmetic on it produces
garbage — `"0" + 1` is `"01"`. Numbers and booleans always go in braces:
`make_section={false}`, not `make_section="false"` (which is a non-empty string
and therefore counts as true).

**A missing brace or comma breaks the whole panel, not one line.** There is no
partial render. If the file will not compile the build fails; if it compiles but
throws at runtime, the panel comes up blank. See the troubleshooting table in
section 7.

**Nothing you change appears until the RUI is rebuilt.** The browser serves a
compiled bundle. Editing a `.js` file on disk changes nothing until
`build_nepi_rui.sh` runs. Python edits need the node restarted, not the RUI
rebuilt. See section 6.

**A `.js` basename is global across every app.** Every app's `rui/` directory
installs *flat* into the same `nepi_rui` source folder. If another app ships a
file with the same basename, one silently overwrites the other. Keep every file
in this directory prefixed `NepiAppControlsSandbox-`.

**`nepi_interfaces` field names are matched by string at runtime.** If a field
is renamed in a `.msg` file, JavaScript does not fail at build time — it
silently reads `undefined`. Search the JavaScript whenever you touch a message.

---

## 5. Editing Controls

Everything in this section is about the **CONTROLS SANDBOX** box, drawn by
`rui/NepiAppControlsSandbox-Controls.js` and declared by
`createControlsInitDict()` in `scripts/controls_sandbox_app_node.py`.

### 5.0 Where a control comes from and where its value goes

A control is one entry in a Python dict:

```python
        'demo_int': {
            'type': 'Int', 'default': 5, 'bounds': [0, 10],
            'display_name': 'Demo Int', 'description': 'Integer value within [0, 10].', 'hidden': False},
```
*(`controls_sandbox_app_node.py:214-216`)*

The dict key — `demo_int` — is the control's **name**. It is what the RUI sends
back in the `name` field of every update message, and what you pass to
`get_control_value()` on the node side. It is the identity of the control. The
rest of the entry is configuration.

The whole dict goes to one `ControlsIF` in `__init__`:

```python
    self.controls_if = ControlsIF(
                    controls_name = 'controls',
                    controls_display_name = 'Controls Sandbox',
                    controls_description = 'One control of every supported type',
                    controls_init_dict = controls_init_dict,
                    controls_updated_callback = self.controlsUpdatedCb,
```
*(`controls_sandbox_app_node.py:131-136`)*

`controls_name = 'controls'` is what puts every control topic under
`.../app_controls_sandbox/controls/`. Change that string and the whole
namespace moves; the RUI follows automatically, because the node reports the
resolved namespace in `ControlsSandboxStatus.controls_namespace` and
`getControlsNamespace()` prefers it over the fallback path.

Ten types are legal. From `nepi_sdk/nepi_controls.py`:

```
CONTROL_TYPES = ["Menu","Selection","Selections","Trigger","Bool", "String", "Int","Float","FloatSlider","FloatSliders"]
```

**A type that is not on that list is silently dropped.** So is an entry that
raises while being built — `create_controls_dict()` wraps each entry in a bare
`try/except: pass`. If a control does not appear in the RUI and there is no
error in the log, a typo in `'type'` is the first thing to check.

### 5.1 Change an existing control's on-screen label

The caption is `display_name`. It is Python. Change the string, restart the
node, done — **no RUI rebuild, no JavaScript edit.**

Before, in `createControlsInitDict()`:

```python
        'demo_int': {
            'type': 'Int', 'default': 5, 'bounds': [0, 10],
            'display_name': 'Demo Int', 'description': 'Integer value within [0, 10].', 'hidden': False},
```

After:

```python
        'demo_int': {
            'type': 'Int', 'default': 5, 'bounds': [0, 10],
            'display_name': 'Scan Count', 'description': 'Integer value within [0, 10].', 'hidden': False},
```

**Do not change the dict key** (`demo_int`) when all you want is a new caption.
The key is the wire identity: it is the `name` field in every update message and
the argument to `get_control_value()`. Renaming it is a different operation —
see 5.4.

The RUI falls back to the key when `display_name` is empty, which is why an
un-labelled control shows as `demo_int` rather than as a blank row:

```jsx
    const display_name = (control_msg.display_name && control_msg.display_name !== '') ? control_msg.display_name : name
```
*(`NepiAppControlsSandbox-Controls.js:252`)*

`description` is the second string in the entry. For slider types it becomes the
hover tooltip (`tooltip={control_msg.description}`). For the other types nothing
currently displays it — the `Input`, `Select` and `AsyncToggle` branches do not
pass it. If you want tooltips on those too, add `title={control_msg.description}`
to the widget; `Input` and `Select` both forward unknown props straight to the
underlying HTML element, where `title` is the tooltip attribute. The Data box
already does exactly this at `NepiAppControlsSandbox-Data.js:180`.

> **Note on the Settings box.** The CONTROLS SETTINGS panel offers a Display
> Name field that looks like it does this at runtime. It publishes to
> `.../controls/set_control_display_name`, and **`ControlsIF` does not subscribe
> that topic.** The same is true of `set_control_description`,
> `set_control_move`, `set_control_reset` and `set_control_factory_reset`. Only
> `set_control_hidden` has a subscriber, and runtime hiding is itself broken
> (see the README's Known rough edges). Treat the Settings box as
> non-functional for now and edit the Python.

### 5.2 Change a control's limits, step size, units or default

**Default and bounds are Python.** Both live in the same dict entry. Before:

```python
        'demo_float_slider': {
            'type': 'FloatSlider', 'default': 50.0, 'bounds': [0.0, 100.0], 'round_value': 1,
            'display_name': 'Demo Float Slider', 'description': 'Single-value slider over [0, 100].', 'hidden': False},
```
*(`controls_sandbox_app_node.py:222-224`)*

After — a slider from -20 to 20 starting at 0, rounded to two decimals:

```python
        'demo_float_slider': {
            'type': 'FloatSlider', 'default': 0.0, 'bounds': [-20.0, 20.0], 'round_value': 2,
            'display_name': 'Demo Float Slider', 'description': 'Single-value slider over [-20, 20].', 'hidden': False},
```

`bounds` is `[min, max]`. Use `-999` in either slot to mean *no limit*; that
sentinel is understood on both sides. `round_value` is how many decimals the
stored value is rounded to; `-1` means no rounding.

**Two bounds traps, both confirmed in `nepi_sdk/nepi_controls.py`:**

- **Keep every `Float` and `FloatSlider` default inside its own bounds.** The
  upper-bound clamp is written `value = float(int_bounds[1])` — the *Int*
  bounds, a copy-paste leftover. A default above its max picks up a stale bound
  from a previously processed control, or raises `NameError`, in which case the
  control is silently dropped.
- **`ControlsIF.set_control_bounds()` calls `set_control_options()`.** Changing
  bounds at runtime from Python writes the string-options field instead. Set
  bounds in the init dict.

**Step size is JavaScript, and only for `FloatSlider`.** `SliderAdjustment`
takes a `step` prop, defaulting to `1`. This app never passes it, so every
`FloatSlider` steps by 1 whatever its range — which makes a 0-to-1 slider
useless. Add the prop in the `FloatSlider` branch.

Before, `NepiAppControlsSandbox-Controls.js:379-393`:

```jsx
      return (
        <SliderAdjustment
          key={name}
          title={display_name}
          comp_name={name}
          topic={namespace + "/set_floatslider_control_value"}
          msgType={"std_msgs/Float32"}
          adjustment={control_msg.set_float}
          min={min}
          max={max}
          scaled={1}
          tooltip={control_msg.description}
          unit={""}
        />
      )
```

After — steps of 0.1, two decimals shown in the read-out box:

```jsx
      return (
        <SliderAdjustment
          key={name}
          title={display_name}
          comp_name={name}
          topic={namespace + "/set_floatslider_control_value"}
          msgType={"std_msgs/Float32"}
          adjustment={control_msg.set_float}
          min={min}
          max={max}
          step={0.1}
          displayDecimals={2}
          scaled={1}
          tooltip={control_msg.description}
          unit={""}
        />
      )
```

This changes **every** `FloatSlider` in the app, because there is one branch for
the type. To give one control its own step you must branch on the name inside
the block — see 5.7.

**Units are JavaScript, and only for the two slider types.** `SliderAdjustment`
and `RangeAdjustment` both take a `unit` prop, and this app passes `unit={""}`
in both places. Fill it in and the suffix appears in the read-out box:

```jsx
          unit={" m"}
```
*(replacing `NepiAppControlsSandbox-Controls.js:391`)*

`Input`, `Select` and `AsyncToggle` have **no** unit prop. For those types the
only place a unit can go is the label, in Python:

```python
            'display_name': 'Scan Count (m)',
```

That is the house answer, not a workaround — the Data box does the same thing.

> **Note.** `msgType={"std_msgs/Float32"}` in the `FloatSlider` block is inert.
> `SliderAdjustment`'s internal `sendUpdate()` checks `comp_name` first, and
> when it is set — as it is here — it calls
> `props.ros.sendUpdateFloatMsg(props.topic, props.comp_name, new_value)` and
> never looks at `msgType`. The prop is harmless; do not spend time changing it.

### 5.3 Change which ROS topic or field a control writes to

Three different questions hide inside this one. Answer them in this order.

**"Which control does this message apply to?"** — the `name` field of the update
message, which is the dict key. Every `sendUpdate*Msg` call in this file passes
`name` as its second argument:

```jsx
            onChange={(e) => sendUpdateIntMsg(namespace + "/set_menu_control_value", name, parseInt(e.target.value, 10))}
```
*(`NepiAppControlsSandbox-Controls.js:267`)*

To repoint an update at a different control, rename the dict key in Python. The
RUI picks the new name up from the status message with no edit at all.

**"Which topic does it go to?"** — determined by the control's *type*, not by
its name. Each branch of `renderControl()` hardcodes one topic suffix:

| Type | Topic suffix | Message type |
| --- | --- | --- |
| `Menu` | `/set_menu_control_value` | `nepi_interfaces/UpdateInt` |
| `Selection` | `/set_selection_control_value` | `nepi_interfaces/UpdateString` |
| `Selections` | `/set_selections_control_value` | `nepi_interfaces/UpdateStringArray` |
| `Trigger` | `/set_trigger_control_value` | `nepi_interfaces/UpdateString` *(mismatched — see below)* |
| `Bool` | `/set_bool_control_value` | `nepi_interfaces/UpdateBool` |
| `String` | `/set_string_control_value` | `nepi_interfaces/UpdateString` |
| `Int` | `/set_int_control_value` | `nepi_interfaces/UpdateInt` |
| `Float` | `/set_float_control_value` | `nepi_interfaces/UpdateFloat` |
| `FloatSlider` | `/set_floatslider_control_value` | `nepi_interfaces/UpdateFloat` |
| `FloatSliders` | `/set_floatsliders_control_value` | `nepi_interfaces/UpdateRangeWindow` |

So **the ordinary way to change the topic a control writes to is to change its
declared type in Python.** Change `'type': 'Int'` to `'type': 'Float'` and the
control moves from `/set_int_control_value` to `/set_float_control_value`,
switches from `set_int` to `set_float` on the status side, and starts parsing
with `parseFloat` instead of `parseInt` — one edit, all three follow.

Editing the topic string in the JSX instead is almost always wrong. `ControlsIF`
subscribes exactly the ten topics above; publish to any other name under that
namespace and nothing is listening.

**"How do I send something `ControlsIF` doesn't handle?"** — give it its own
plain app topic and bypass `ControlsIF` entirely. This is the sanctioned
pattern, and it is the fix for the broken `Trigger` type: the Controls box sends
an `UpdateString`, `ControlsIF` subscribes the topic as `UpdateTrigger`, and the
types do not match, so **a Trigger control cannot be fired from the RUI.**

`nepi_app_stereo_cam` does it this way. In the RUI:

```jsx
  onReloadProcesses() {
    const { sendTriggerMsg } = this.props.ros
    const namespace = this.props.appNamespace
    if (namespace != null) {
      sendTriggerMsg(namespace + "/reload_processes")
    }
  }
```
*(`nepi_app_stereo_cam/rui/NepiAppStereoCam-Controls.js:86-92`)*

and in the node's `SUBS_DICT`:

```python
            'reload_processes': {
                'namespace': self.node_namespace,
                'topic': 'reload_processes',
                'msg': Empty,
                'qsize': 10,
                'callback': self.reloadProcessesCb,
                'callback_args': ()
            }
```
*(`nepi_app_obstacles/scripts/obstacles_app_node.py:310-317`)*

`sendTriggerMsg` publishes a `std_msgs/Empty`. This app's `SUBS_DICT` at
`controls_sandbox_app_node.py:104-112` currently holds one entry
(`system_status`); a new plain topic is added alongside it.

**Changing the namespace itself.** All ten topics sit under whatever
`controls_name` was passed to the `ControlsIF` constructor. There is one string
to change, at `controls_sandbox_app_node.py:132`, and nothing in the RUI needs
to follow it.

### 5.4 Add a brand new control, end to end

**A new control of an existing type is one Python edit. No JavaScript, no
message change, no params change, no RUI rebuild.**

That is not a simplification — it is the whole point of the generic renderer.
Work through it once and the pattern is obvious.

Goal: a "Scan Rate" float slider, 0.5 to 10 Hz, defaulting to 2 Hz, that makes
the node log the new rate.

#### File 1 of 1 — `scripts/controls_sandbox_app_node.py`

**Before** (the end of `createControlsInitDict`, lines 226-230):

```python
        'demo_floats_slider': {
            'type': 'FloatSliders', 'default': [0.25, 0.75], 'bounds': [0.0, 1.0], 'round_value': 2,
            'display_name': 'Demo Floats Slider', 'description': 'Dual-value range slider (0.0-1.0 ratio).', 'hidden': False},
    }
    return controls_init_dict
```

**After:**

```python
        'demo_floats_slider': {
            'type': 'FloatSliders', 'default': [0.25, 0.75], 'bounds': [0.0, 1.0], 'round_value': 2,
            'display_name': 'Demo Floats Slider', 'description': 'Dual-value range slider (0.0-1.0 ratio).', 'hidden': False},

        'scan_rate': {
            'type': 'FloatSlider', 'default': 2.0, 'bounds': [0.5, 10.0], 'round_value': 1,
            'display_name': 'Scan Rate (Hz)', 'description': 'Scan updates per second.', 'hidden': False},
    }
    return controls_init_dict
```

Note the comma after the previous entry's closing brace. Insertion order sets
the initial display order, so a new entry lands at the bottom of the box.

That is the entire change. Restart the node and the slider is on screen,
labelled, bounded, and publishing to
`.../controls/set_floatslider_control_value` with `name: "scan_rate"`.

#### Making it *do* something

A control that only stores a number is not much use. Read it in
`controlsUpdatedCb`, which fires after every applied change.

**Before** (`controls_sandbox_app_node.py:310-315`):

```python
  def controlsUpdatedCb(self, control_name):
    # Called by ControlsIF after a control value/display change is applied.
    value = None
    if self.controls_if is not None:
      value = self.controls_if.get_control_value(control_name)
    self.msg_if.pub_info("Control '" + str(control_name) + "' updated to: " + str(value))
```

**After:**

```python
  def controlsUpdatedCb(self, control_name):
    # Called by ControlsIF after a control value/display change is applied.
    value = None
    if self.controls_if is not None:
      value = self.controls_if.get_control_value(control_name)
    self.msg_if.pub_info("Control '" + str(control_name) + "' updated to: " + str(value))
    if control_name == 'scan_rate':
      self.scan_rate = float(value)
```

`controlsUpdatedCb` is `camelCase` with a `Cb` suffix: by the NEPI convention
that marks it private and a ROS callback, so it takes no docstring, and it is
registered with `ControlsIF` at construction. **Renaming it requires auditing
every external call site first** — here, the `controls_updated_callback =
self.controlsUpdatedCb` argument at `controls_sandbox_app_node.py:136`. Adding
to its body, as above, carries no such risk.

To read a value at any other moment, call `get_control_value()` directly. It is
`snake_case` and public, so it is part of the documented API surface:

```python
    rate = self.controls_if.get_control_value('scan_rate')
```

#### Do I need to touch `ControlsSandboxStatus.msg`?

**No.** This is the question that catches people. `ControlsSandboxStatus` is
this app's own five-field status message:

```
string name
string controls_namespace
bool controls_ready
string data_namespace
bool data_ready
```
*(`msg/ControlsSandboxStatus.msg`)*

It says where the interfaces live and whether they are up. Per-control data
travels in `nepi_interfaces/ControlsStatus`, which `ControlsIF` publishes and
which already has room for any number of controls in its
`controls_name_list` / `controls_type_list` / `controls_msg_list` arrays.

You add a field to `ControlsSandboxStatus.msg` only when the *app itself* needs
to report something that is neither a control nor a datum. If you do, add it to
the `.msg`, populate it in `publish_status()` at
`controls_sandbox_app_node.py:354-374`, and rebuild the workspace — a `.msg`
change requires `catkin`, not just a node restart. `CMakeLists.txt` already
lists the file under `add_message_files`, so no build-file edit is needed.

#### Do I need to touch `controls_sandbox_app_params.yaml`?

**No.** This app declares no app-level params:

```python
    # This app carries no app-level params of its own; all persisted control
    # state is managed by the ControlsIF instance under <node>/controls, and all
    # persisted datum display state by the DataIF instance under <node>/data.
    self.PARAMS_DICT = dict()
```
*(`controls_sandbox_app_node.py:87-90`)*

`ControlsIF` registers one param of its own, `/controls_controls_dict`, and
writes the whole controls dict into it on every change, so control values
already survive a node restart with no YAML entry. The only reason to edit
`controls_sandbox_app_params.yaml` is to add a filename to
`RUI_DICT.rui_files`, or to change the app's display name or group.

#### When you *do* have to touch the JavaScript

Only when the control type you need does not exist yet — an eleventh entry in
`CONTROL_TYPES`. That is a change to `nepi_sdk/nepi_controls.py`,
`nepi_interfaces/Control.msg`, `nepi_api/system_if.py` **and** a new branch in
`renderControl()`, and it is outside the scope of this guide. If you are
reaching for it, check first whether an existing type will do: `Menu` covers
"pick one of N", `Selections` covers "pick any of N", `FloatSlider` covers any
bounded number.

### 5.5 Remove a control safely

**Delete the dict entry. That is the whole RUI-side removal.**

In `createControlsInitDict()`, delete:

```python
        'demo_string': {
            'type': 'String', 'default': 'hello nepi',
            'display_name': 'Demo String', 'description': 'Free-form text value.', 'hidden': False},
```
*(`controls_sandbox_app_node.py:210-212`)*

The control leaves `controls_name_list`, the `.map()` at
`NepiAppControlsSandbox-Controls.js:458` stops producing a row for it, and the
widget disappears. **Nothing in `NepiAppControlsSandbox-Controls.js` needs to
change**, because nothing in it names that control.

**What becomes dead in the node.** Search for the key before you delete:

```bash
grep -rn "demo_string" scripts/ rui/ params/ msg/
```

Anything that names it — a branch in `controlsUpdatedCb`, a
`get_control_value('demo_string')` call elsewhere, a stored attribute — is now
dead and should go with it. A `get_control_value()` on a deleted key returns
`None`, silently, which is exactly the kind of bug that shows up two weeks
later as a `TypeError` on `float(None)`.

**What must be left alone.**

- **The `String` branch in `renderControl()`** — lines 353-370. It is shared by
  `String`, `Int` and `Float`. Deleting it because "the string control is gone"
  takes the Int and Float controls with it.
- **The `onInputChange` / `onInputKey` helpers** — lines 202-244. Same reason:
  every typed control uses them.
- **`getControlValue()`** — lines 106-118. Used by the pending-edit
  reconciliation in `statusListener()`.
- **`ControlsSandboxStatus.msg`** — nothing in it is per-control. Never edit it
  when removing a control.
- **The persisted param.** `ControlsIF` writes the controls dict to
  `/controls_controls_dict` on the param server, and `config_mgr` may have that
  on disk with the deleted control still in it. `init()` reads the param back
  and overwrites the freshly-built dict, so a stale saved config can resurrect a
  deleted control after a restart. If a control you deleted is still on screen,
  factory-reset the node's config before assuming the delete failed.

**Removing the last control of a type is not a reason to delete its branch.**
The branches are generic. Leaving an unused branch in place costs nothing and
keeps the file a complete reference to the ten types; deleting one means the
next person who declares that type gets a silently blank row, because
`renderControl()` ends with a bare `return null` for an unrecognised type
(line 420).

### 5.6 Worked walkthrough of each widget type

One subsection per widget this app actually renders. Each gives the exact
existing code, and the one or two lines you change. All line numbers are in
`rui/NepiAppControlsSandbox-Controls.js` unless stated otherwise.

Remember: **you almost never edit these.** They are here for when you want to
change how a whole type looks, or when you need to special-case one control by
name. The normal way to get a widget on screen is a dict entry in Python.

#### 5.6.1 Dropdown, index-valued — `Menu`

Declared as (`controls_sandbox_app_node.py:190-192`):

```python
        'demo_menu': {
            'type': 'Menu', 'default': 1, 'options': ['Off', 'Low', 'High'],
            'display_name': 'Demo Menu', 'description': 'Pick one menu option (index based).', 'hidden': False},
```

Rendered by (lines 259-273):

```jsx
    if (type === "Menu") {
      const options = control_msg.string_options
      const set_index = control_msg.set_index
      return (
        <Label title={display_name} key={name}>
          <Select
            id={'csbx_' + name}
            value={set_index}
            onChange={(e) => sendUpdateIntMsg(namespace + "/set_menu_control_value", name, parseInt(e.target.value, 10))}
          >
            {options.map((opt, i) => <Option key={name + '_' + i} value={i}>{opt}</Option>)}
          </Select>
        </Label>
      )
    }
```

The value on the wire is the **position** of the choice, not its text. Picking
"High" sends `2`.

*What you change:* the options and the default, both in Python. To make the
dropdown wider, add `style={{ width: "100%" }}` to the `<Select>` — it forwards
unknown props to the HTML `<select>`.

#### 5.6.2 Dropdown, text-valued — `Selection`

Declared as (`controls_sandbox_app_node.py:194-196`):

```python
        'demo_selection': {
            'type': 'Selection', 'default': 'Bravo', 'options': ['Alpha', 'Bravo', 'Charlie'],
            'display_name': 'Demo Selection', 'description': 'Select a single option by name.', 'hidden': False},
```

Rendered by (lines 277-291):

```jsx
    if (type === "Selection") {
      const options = control_msg.string_options
      const set_string = control_msg.set_string
      return (
        <Label title={display_name} key={name}>
          <Select
            id={'csbx_' + name}
            value={set_string}
            onChange={(e) => sendUpdateStringMsg(namespace + "/set_selection_control_value", name, e.target.value)}
          >
            {options.map((opt, i) => <Option key={name + '_' + i} value={opt}>{opt}</Option>)}
          </Select>
        </Label>
      )
    }
```

Identical shape to `Menu`, one difference: `value={opt}` on the `<Option>`
rather than `value={i}`, so the wire carries `"Bravo"` rather than `1`.

*Choosing between them:* use `Selection` when the option strings are the real
values (topic names, mode names) and `Menu` when they are labels for positions.
`Selection` rejects any value not in `options`; changing the option list without
changing the default leaves the control on `options[0]`.

#### 5.6.3 Multi-select — `Selections`

Declared as (`controls_sandbox_app_node.py:198-200`):

```python
        'demo_selections': {
            'type': 'Selections', 'default': ['Red', 'Blue'], 'options': ['Red', 'Green', 'Blue'],
            'display_name': 'Demo Selections', 'description': 'Select any number of options.', 'hidden': False},
```

Rendered by (lines 296-321):

```jsx
    if (type === "Selections") {
      const options = control_msg.string_options
      const set_strings = control_msg.set_strings || []
      const { sendUpdateStringArrayMsg } = this.props.ros
      return (
        <Label title={display_name} key={name}>
          <div>
            {options.map((opt, i) => (
              <div key={name + '_' + i} style={{ display: "inline-block", marginRight: Styles.vars.spacing.regular, textAlign: "center" }}>
                <div style={{ fontSize: Styles.vars.fontSize.small, marginBottom: Styles.vars.spacing.xs }}>{opt}</div>
                <AsyncToggle
                  checked={set_strings.indexOf(opt) !== -1}
                  onClick={() => {
                    // Send the complete desired selection (declarative), not a toggle.
                    const next = set_strings.indexOf(opt) !== -1
                      ? set_strings.filter((s) => s !== opt)
                      : [...set_strings, opt]
                    sendUpdateStringArrayMsg(namespace + "/set_selections_control_value", name, next)
                  }}
                />
              </div>
            ))}
          </div>
        </Label>
      )
    }
```

One toggle per option, laid out side by side by `display: "inline-block"`. Each
click sends the **complete** desired list, not a single change — so a dropped
message costs one click, not a wrong state.

*What you change:* the option list, in Python. To stack the toggles vertically
instead, change `display: "inline-block"` to `display: "block"` on line 304.

#### 5.6.4 Button — `Trigger`

Declared as (`controls_sandbox_app_node.py:202-204`):

```python
        'demo_trigger': {
            'type': 'Trigger', 'default': 0,
            'display_name': 'Demo Trigger', 'description': 'Fire a one-shot trigger.', 'hidden': False},
```

Rendered by (lines 325-333):

```jsx
    if (type === "Trigger") {
      return (
        <Label title={display_name} key={name}>
          <ButtonMenu>
            <Button onClick={() => sendUpdateStringMsg(namespace + "/set_trigger_control_value", name, "")}>{"Trigger"}</Button>
          </ButtonMenu>
        </Label>
      )
    }
```

`<ButtonMenu>` is a right-aligned row that spaces its buttons; a single button
does not need it, but the house style keeps it so a second button can be added
without restyling.

**This widget does not work.** The click publishes a `nepi_interfaces/UpdateString`
and `ControlsIF` subscribes the topic as `nepi_interfaces/UpdateTrigger`. The
types disagree, so the node never receives it, and `Store.js` has no
`sendUpdateTriggerMsg` to send the right type with. Use the plain-app-topic
pattern in 5.3 instead. The `Trigger` entry stays in the dict for completeness
of the type survey.

To change the button caption, change the literal text — `{"Trigger"}` on line
329 — not the label. `display_name` sets the caption on the left; this sets the
text on the button itself.

#### 5.6.5 Toggle — `Bool`

Declared as (`controls_sandbox_app_node.py:206-208`):

```python
        'demo_bool': {
            'type': 'Bool', 'default': True,
            'display_name': 'Demo Bool', 'description': 'Toggle a boolean on or off.', 'hidden': False},
```

Rendered by (lines 337-347):

```jsx
    if (type === "Bool") {
      const checked = (control_msg.set_bool === true)
      return (
        <Label title={display_name} key={name}>
          <AsyncToggle
            checked={checked}
            onClick={() => sendUpdateBoolMsg(namespace + "/set_bool_control_value", name, !checked)}
          />
        </Label>
      )
    }
```

`AsyncToggle`, not `react-toggle` — this is the platform-wide rule for any
toggle whose `checked` reads a backend value. The thumb moves the moment you
click; **the colour only follows a confirmed value from the node.** So a thumb
that has moved with the colour not yet following means the request is in flight,
and a thumb that springs back after three seconds means nothing confirmed it.

That distinction matters here, because `ControlsIF.set_control_value()` writes
bare `self.publish_status` with no parentheses — the call never happens — so
confirmation waits out the 1 Hz status timer. Up to a second of
thumb-moved-colour-not-yet is normal for this app and is not a fault.

The one place plain `react-toggle` is still correct is the Show Controls switch
at lines 437-440, whose `checked` is local view state with no round trip. It
carries a comment saying so. Follow that convention if you add another
view-preference toggle.

#### 5.6.6 Typed entry box — `String`, `Int`, `Float`

Declared as (`controls_sandbox_app_node.py:210-220`, three entries):

```python
        'demo_string': {
            'type': 'String', 'default': 'hello nepi',
            'display_name': 'Demo String', 'description': 'Free-form text value.', 'hidden': False},

        'demo_int': {
            'type': 'Int', 'default': 5, 'bounds': [0, 10],
            'display_name': 'Demo Int', 'description': 'Integer value within [0, 10].', 'hidden': False},

        'demo_float': {
            'type': 'Float', 'default': 2.5, 'bounds': [0.0, 10.0], 'round_value': 2,
            'display_name': 'Demo Float', 'description': 'Float value within [0.0, 10.0].', 'hidden': False},
```

All three share one branch (lines 353-370):

```jsx
    if (type === "String" || type === "Int" || type === "Float") {
      var msgValue = ''
      if (type === "String") { msgValue = control_msg.set_string }
      else if (type === "Int") { msgValue = control_msg.set_int }
      else { msgValue = control_msg.set_float }
      const value = editing ? this.state.editValues[name] : msgValue
      return (
        <Label title={display_name} key={name}>
          <Input
            id={'csbx_' + name}
            style={{ width: "100%" }}
            value={value}
            onChange={(e) => this.onInputChange(name, e)}
            onKeyDown={(e) => this.onInputKey(name, type, e)}
          />
        </Label>
      )
    }
```

This is the canonical NEPI editable-input pattern and the most intricate thing
in the file. Four rules make it work; keep all four if you copy it:

1. **`id` is required.** `onInputChange` and `onInputKey` find the element with
   `document.getElementById('csbx_' + name)` in order to restyle it. Drop the
   `id` and the red-while-editing styling silently stops.
2. **Typing does not send.** `onChange` (lines 202-208) only stores the text in
   `this.state.editValues` and turns it red via `setElementStyleModified`.
3. **Enter sends.** `onKeyDown` (lines 210-244) parses the text to the declared
   type, publishes, clears the red, and records a `pending` entry.
4. **The typed text stays on screen until the node confirms it.**
   `statusListener` (lines 120-164) drops the override once the reported value
   either equals what you typed or has moved off what it was — which is what
   makes a clamp visible. Type `99` into Demo Int, whose bounds are `[0, 10]`,
   and the box shows `99` until the node reports `10`.

*What you change:* box width, on line 362. Everything else about these controls
is Python. Never make `onChange` send — it would publish one ROS message per
keystroke.

#### 5.6.7 Slider — `FloatSlider`

Covered in full in 5.2, including the `step`, `displayDecimals` and `unit`
props. Declared at `controls_sandbox_app_node.py:222-224`, rendered at lines
375-394.

The bounds handling is worth reading once:

```jsx
      const bounds = control_msg.float_bounds || []
      const min = (bounds.length > 0 && bounds[0] !== -999) ? bounds[0] : 0
      const max = (bounds.length > 1 && bounds[1] !== -999) ? bounds[1] : 100
```
*(lines 376-378)*

`|| []` means "use an empty list if this field is missing" — the guard against a
message that arrives before the node has filled it in. `-999` is the no-limit
sentinel, and a slider needs *some* limit to draw, so it falls back to 0-100.
**A `FloatSlider` declared without `bounds` silently becomes a 0-100 slider**,
whatever its real range.

#### 5.6.8 Range slider — `FloatSliders`

Declared at `controls_sandbox_app_node.py:226-228`, rendered at lines 399-418:

```jsx
    if (type === "FloatSliders") {
      const set_floats = control_msg.set_floats || [0, 1]
      const bounds = control_msg.float_bounds || []
      const min_limit = (bounds.length > 0 && bounds[0] !== -999) ? bounds[0] : 0
      const max_limit = (bounds.length > 1 && bounds[1] !== -999) ? bounds[1] : 1
      return (
        <RangeAdjustment
          key={name}
          title={display_name}
          comp_name={name}
          topic={namespace + "/set_floatsliders_control_value"}
          min={set_floats[0]}
          max={set_floats[1]}
          min_limit_m={min_limit}
          max_limit_m={max_limit}
          tooltip={control_msg.description}
          unit={""}
        />
      )
    }
```

Two handles, one for each end of a range. `min`/`max` are the current handle
positions as ratios between 0 and 1; `min_limit_m`/`max_limit_m` are the real
values those ratios map onto for display.

**Avoid this type.** It is broken in three places in `nepi_sdk/nepi_controls.py`
— the `create_controls_dict` branch clamps into an undefined `value[0]`, reads
an undefined `int_bounds`, and `set_control_value` reads `value0` before
assigning it. The exceptions are swallowed by the bare `try/except`, so the
control is dropped rather than erroring. `demo_floats_slider` is in the dict for
completeness of the type survey, not because it works. Use two `FloatSlider`
controls until the SDK is fixed.

#### 5.6.9 Special-casing one control by name

Everything above changes a whole type. When you need one control to differ,
branch on its name *inside* the type block, before the shared `return`:

```jsx
    if (type === "FloatSlider") {
      const bounds = control_msg.float_bounds || []
      const min = (bounds.length > 0 && bounds[0] !== -999) ? bounds[0] : 0
      const max = (bounds.length > 1 && bounds[1] !== -999) ? bounds[1] : 100
      // Scan Rate is the only slider fine enough to need sub-unit steps.
      const step = (name === 'scan_rate') ? 0.1 : 1
      return (
        <SliderAdjustment
          key={name}
          title={display_name}
          comp_name={name}
          topic={namespace + "/set_floatslider_control_value"}
          msgType={"std_msgs/Float32"}
          adjustment={control_msg.set_float}
          min={min}
          max={max}
          step={step}
          scaled={1}
          tooltip={control_msg.description}
          unit={""}
        />
      )
    }
```

Do this sparingly and comment the reason. A name check in the RUI is a hidden
coupling between a JavaScript file and a Python dict key: rename the key and the
special case silently stops applying, with no error anywhere.

---

## 6. Editing Data

Everything in this section is about the **DATA SANDBOX** box, drawn by
`rui/NepiAppControlsSandbox-Data.js` and declared by `createDataInitDict()` in
`scripts/controls_sandbox_app_node.py`.

The Data box is the Controls box with the arrow reversed. Same generic-renderer
structure, same three parallel arrays, same Python-owned declaration — but
**nothing in it ever publishes**. The file has no `sendUpdate*` call, no
editable input, and no writer of any kind:

```jsx
// This component is display only. It has no publishers, no editable inputs and
// no calls into any Store.js send function. The node that owns the DataIF is
// the only writer of record. The single Toggle below drives local component
// state ("Show Data") and never touches ROS.
```
*(`NepiAppControlsSandbox-Data.js:55-58`)*

Keep it that way. If a value needs to be operator-settable it is a control, not
a datum.

### 6.1 Where the Data panel's values come from

**The topic:** `/nepi/<device>/app_controls_sandbox/data/status`.
**The message:** `nepi_interfaces/DataStatus`.

The subscription is set up here:

```jsx
      var statusListener = this.props.ros.setupStatusListener(
        statusNamespace,
        "nepi_interfaces/DataStatus",
        this.statusListener
      )
```
*(`NepiAppControlsSandbox-Data.js:106-110`)*

`statusNamespace` is the `namespace` prop plus `/status`, and the `namespace`
prop is handed down by the page from `getDataNamespace()`
(`NepiAppControlsSandbox.js:71-79`), which prefers
`ControlsSandboxStatus.data_namespace` and falls back to `<app>/data`.

**From message field to line on screen, in four steps.**

`DataStatus` carries four parallel arrays. Position `i` in each describes the
same datum:

```
string[] data_name_list
string[] data_type_list
nepi_interfaces/Datum[] data_msg_list
bool[] data_hidden_list
```
*(`nepi_interfaces/msg/DataStatus.msg`)*

1. **The arrays are unpacked** in `render()`:

   ```jsx
      const names = status_msg.data_name_list || []
      const types = status_msg.data_type_list || []
      const msgs = status_msg.data_msg_list || []
      const hiddens = status_msg.data_hidden_list || []
   ```
   *(`NepiAppControlsSandbox-Data.js:243-246`)*

2. **One row is produced per name**, in list order, skipping hidden ones:

   ```jsx
            {names.map((name, i) => {
              const datum_msg = msgs[i]
              if (datum_msg == null) { return null }
              // Hidden data are not shown in the Data box.
              if (hiddens[i] === true || datum_msg.hidden === true) { return null }
              return this.renderDatum(name, types[i], datum_msg, i)
            })}
   ```
   *(`NepiAppControlsSandbox-Data.js:250-256`)*

3. **`renderDatum()` picks a widget by type** — line 139, one `if` per type,
   exactly as `renderControl()` does.

4. **The value is read from the type's field on the `Datum` message.** A
   `Datum` has one field per type and only the matching one is populated:

   | Type | Field read | Widget |
   | --- | --- | --- |
   | `Bool` | `value_bool` | `BooleanIndicator` |
   | `Bools` | `value_bools` | one `BooleanIndicator` per element |
   | `String` | `value_string` | disabled `Input` |
   | `Strings` | `value_strings` | one disabled `Input` per element |
   | `Int` | `value_int` | disabled `Input` |
   | `Ints` | `value_ints` | one disabled `Input` per element |
   | `Float` | `value_float`, rounded | disabled `Input` |
   | `Floats` | `value_floats`, each rounded | one disabled `Input` per element |

**Where the values are written.** `dataUpdaterCb()` runs once per second and
writes all eight:

```python
    self.data_if.set_datum_value('demo_bool_data', toggle)
    self.data_if.set_datum_value('demo_bools_data', [toggle, not toggle])
    self.data_if.set_datum_value('demo_string_data', stamp)
```
*(`controls_sandbox_app_node.py:334-336`, first three of eight)*

`set_datum_value()` is `snake_case` and public: it writes the value, stamps its
timestamp, and **publishes status on every call**. That is why `dataUpdaterCb()`
ends with `return False` — there is nothing left for the updater loop to
publish. `dataUpdaterCb` is `camelCase` with a `Cb` suffix, so it is private and
takes no docstring; it is registered as `data_updater_callback` at
`controls_sandbox_app_node.py:159` and renaming it requires auditing that call
site.

The rate is set by one argument:

```python
                    data_updater_max_rate = 1,
```
*(`controls_sandbox_app_node.py:158`)*

Raise it for a faster display. Every write publishes a full `DataStatus`, so
eight data at 10 Hz is eighty messages a second; do not raise it casually.
`-1` disables the updater thread entirely, for a node that writes its data from
somewhere else.

### 6.2 Rename a displayed data label

Same as a control: `display_name`, in Python, no RUI rebuild.

Before (`controls_sandbox_app_node.py:265-267`):

```python
        'demo_float_data': {
            'type': 'Float', 'value': 0.0, 'round_value': 3, 'round_display': 3,
            'display_name': 'Demo Float', 'description': 'A sine wave over the update counter.', 'hidden': False},
```

After:

```python
        'demo_float_data': {
            'type': 'Float', 'value': 0.0, 'round_value': 3, 'round_display': 3,
            'display_name': 'Heading Error (deg)', 'description': 'A sine wave over the update counter.', 'hidden': False},
```

Do not change the key `demo_float_data` — it is the argument to
`set_datum_value()`. Change it and the write at line 340 silently stops
matching.

The same fallback applies as on the controls side, so an unlabelled datum shows
its key:

```jsx
    const display_name = (datum_msg.display_name && datum_msg.display_name !== '') ? datum_msg.display_name : name
```
*(`NepiAppControlsSandbox-Data.js:140`)*

**`description` does display here.** Unlike most control widgets, every Data
widget passes it through as the hover tooltip — `title={description}` on the
`Input` at line 180 and on the `BooleanIndicator` at line 151. Use it; it is the
cheapest documentation in the app.

### 6.3 Change formatting, units or decimal places

**Decimal places: Python, per datum.** Two different knobs, and the distinction
matters:

```python
        'demo_float_data': {
            'type': 'Float', 'value': 0.0, 'round_value': 3, 'round_display': 3,
```
*(`controls_sandbox_app_node.py:265-266`)*

- **`round_value`** — decimals the *stored* value is rounded to before it goes
  on the wire. Destructive; the precision is gone. `-1` means no rounding.
- **`round_display`** — decimals the *RUI* formats the value to for display.
  Non-destructive; the wire keeps full precision. Defaults to 2.

For a value another node also consumes, leave `round_value` at `-1` and set only
`round_display`. Rounding for the sake of a display is a bad reason to lose
precision on the wire.

The display side reads it here:

```jsx
    // round_display is the number of decimals the RUI formats a Float/Floats
    // value to. Default 2 when the datum does not carry a sane value.
    const decimals = (typeof datum_msg.round_display === 'number' && datum_msg.round_display >= 0) ? datum_msg.round_display : 2
```
*(`NepiAppControlsSandbox-Data.js:143-145`)*

and applies it with `round()` from `Utilities.js`, which is
`Number(value).toFixed(decimals)` — so it **pads as well as truncates**. A
`round_display` of 3 shows `0.500`, not `0.5`. That is usually what you want for
a live readout, because the number stops jittering in width.

**`round_display` applies to `Float` and `Floats` only.** `Int`, `String` and
the bool types ignore it.

**Units: Python, in the label.** No Data widget takes a unit prop — `Input` has
none and `BooleanIndicator` has none. The only place a unit can go is
`display_name`:

```python
            'display_name': 'Heading Error (deg)',
```

That is the convention across the RUI, not a shortcut.

**Other formatting is JavaScript, and changes the whole type.** The one place to
change how every `Float` renders is line 177:

```jsx
      else { value = round(datum_msg.value_float, decimals) }
```

To show a `Float` as a percentage, for instance:

```jsx
      else { value = round(datum_msg.value_float * 100, decimals) + " %" }
```

Both `value` and the appended suffix end up as text in a disabled `Input`, so
appending a string is safe — this box never parses what it displays.

Box width is on line 180:

```jsx
          <Input disabled title={description} style={{ width: "100%" }} value={value} />
```

**Do not remove `disabled`.** It is what makes the box read-only, and it is also
what applies the grey-orange background from `Input.js` that marks a readout as
not-editable. An enabled `Input` here would let an operator type into a box that
is overwritten by the node a second later.

### 6.4 Add a new data readout, end to end

Unlike a control, a datum needs **two** Python edits: one to declare it, one to
write it. Still no JavaScript and still no message change.

Goal: report the node's uptime in seconds.

#### File 1 of 2 — declare it in `createDataInitDict()`

**Before** (`controls_sandbox_app_node.py:269-273`):

```python
        'demo_floats_data': {
            'type': 'Floats', 'value': [0.0, 0.0], 'round_value': 3, 'round_display': 3,
            'display_name': 'Demo Floats', 'description': 'The sine wave and its negation.', 'hidden': False},
    }
    return data_init_dict
```

**After:**

```python
        'demo_floats_data': {
            'type': 'Floats', 'value': [0.0, 0.0], 'round_value': 3, 'round_display': 3,
            'display_name': 'Demo Floats', 'description': 'The sine wave and its negation.', 'hidden': False},

        'uptime_data': {
            'type': 'Float', 'value': 0.0, 'round_value': -1, 'round_display': 1,
            'display_name': 'Uptime (s)', 'description': 'Seconds since the node started.', 'hidden': False},
    }
    return data_init_dict
```

A datum entry is smaller than a control entry: it has `value` where a control
has `default`, and it has **no bounds, no options, no factory value**. A datum
is a value and a timestamp. Insertion order sets the display order, so this
lands at the bottom of the box.

Eight types are legal. From `nepi_sdk/nepi_data.py`:

```
DATUM_TYPES = ["Bool", "Bools", "String", "Strings", "Int", "Ints","Float","Floats"]
```

There is no `Menu`, no `Selection` and no slider datum — those are control
concepts, and `Datum.msg` says so in a comment. As with controls, an unknown
type is silently dropped.

#### File 2 of 2 — write it in `dataUpdaterCb()`

Same file. **Before** (`controls_sandbox_app_node.py:325-343`):

```python
    if self.data_if is None:
      return False

    self.data_counter = self.data_counter + 1
    count = self.data_counter
    sine = math.sin(float(count) / 10.0)
    toggle = (count % 2 == 0)
    stamp = time.strftime('%H:%M:%S')

    self.data_if.set_datum_value('demo_bool_data', toggle)
    self.data_if.set_datum_value('demo_bools_data', [toggle, not toggle])
    self.data_if.set_datum_value('demo_string_data', stamp)
    self.data_if.set_datum_value('demo_strings_data', [stamp, 'tick ' + str(count)])
    self.data_if.set_datum_value('demo_int_data', count)
    self.data_if.set_datum_value('demo_ints_data', [count, -count])
    self.data_if.set_datum_value('demo_float_data', sine)
    self.data_if.set_datum_value('demo_floats_data', [sine, -sine])

    return False
```

**After:**

```python
    if self.data_if is None:
      return False

    self.data_counter = self.data_counter + 1
    count = self.data_counter
    sine = math.sin(float(count) / 10.0)
    toggle = (count % 2 == 0)
    stamp = time.strftime('%H:%M:%S')

    self.data_if.set_datum_value('demo_bool_data', toggle)
    self.data_if.set_datum_value('demo_bools_data', [toggle, not toggle])
    self.data_if.set_datum_value('demo_string_data', stamp)
    self.data_if.set_datum_value('demo_strings_data', [stamp, 'tick ' + str(count)])
    self.data_if.set_datum_value('demo_int_data', count)
    self.data_if.set_datum_value('demo_ints_data', [count, -count])
    self.data_if.set_datum_value('demo_float_data', sine)
    self.data_if.set_datum_value('demo_floats_data', [sine, -sine])
    self.data_if.set_datum_value('uptime_data', nepi_sdk.get_time() - self.start_time)

    return False
```

That needs `self.start_time` to exist. Set it once in `__init__`, next to the
other node identity assignments at `controls_sandbox_app_node.py:65-67`:

```python
    self.node_namespace = nepi_sdk.get_node_namespace()
    self.start_time = nepi_sdk.get_time()
```

**A declared datum that is never written stays at its initial `value` forever
and shows no error.** That is the most common way a new readout goes wrong:
step one done, step two forgotten. If a new row is on screen but frozen, check
`dataUpdaterCb()` first.

Writing from anywhere else is fine — `set_datum_value()` can be called from any
callback, and each call publishes. The 1 Hz updater is a convenience for values
that have to be sampled, not a requirement.

#### Do I need to touch `ControlsSandboxStatus.msg`?

**No** — same answer as for a control, for the same reason. Per-datum data
travels in `nepi_interfaces/DataStatus`, whose arrays already hold any number of
data. `ControlsSandboxStatus` reports only where the interfaces live.

Read 6.5 before you ever change that `.msg` file.

#### Do I need to touch `controls_sandbox_app_params.yaml`?

**No.** `PARAMS_DICT` is empty, and `DataIF` registers its own `data_data_dict`
param for the display state it persists.

### 6.5 Remove a data readout safely

**Two deletions, both Python.** Delete the entry from `createDataInitDict()`:

```python
        'demo_ints_data': {
            'type': 'Ints', 'value': [0, 0],
            'display_name': 'Demo Ints', 'description': 'The counter and its negation.', 'hidden': False},
```
*(`controls_sandbox_app_node.py:261-263`)*

and delete its write from `dataUpdaterCb()`:

```python
    self.data_if.set_datum_value('demo_ints_data', [count, -count])
```
*(`controls_sandbox_app_node.py:339`)*

**Delete both, in that order, in one edit.** A `set_datum_value()` on a datum
that is no longer declared does not raise — `nepi_data.set_datum_value()` looks
the key up in a dict and does nothing when it is absent — so a leftover write is
a silent no-op that will confuse the next reader. It costs a wasted
`publish_status()` per updater cycle.

Then check for other references before you finish:

```bash
grep -rn "demo_ints_data" scripts/ rui/ params/ msg/
```

**What must be left alone.**

- **The `Strings`/`Ints`/`Floats` branch in `renderDatum()`** — lines 187-206.
  One branch serves all three array types. Deleting it because the Ints datum is
  gone takes the Strings and Floats data with it.
- **The `String`/`Int`/`Float` branch** — lines 173-183. Same shared-branch
  problem.
- **`ControlsSandboxStatus.msg`** — nothing in it is per-datum.
- **The persisted param.** `DataIF` writes display state to `data_data_dict`. As
  on the controls side, a stale saved config can put a deleted datum back after
  a restart; factory-reset the config before concluding the delete failed.

#### Before deleting a field from any `.msg` file

Deleting a datum does not touch a `.msg` file, and that is the normal case. But
if you ever do change one — this app's `ControlsSandboxStatus.msg`, or anything
in `nepi_interfaces` — three things are true and all three cost real time when
missed:

1. **A `.msg` change requires a full rebuild, not a node restart.**
   `catkin_make` or `catkin build` regenerates the Python and JavaScript message
   definitions. Until it runs, the node and the RUI are using the old shape.
2. **Removing or reordering a field breaks every other consumer of that
   message.** ROS message compatibility is positional at the wire level and
   nominal in the generated classes; a node built against the old definition and
   one built against the new one will not agree. Everything that uses the
   message must be rebuilt together.
3. **JavaScript will not tell you.** `roslib.js` reads fields by string name.
   Rename `data_namespace` and `getDataNamespace()` starts reading `undefined`,
   silently, with no build error and no runtime exception — the Data box just
   never subscribes and comes up empty.

So before touching a `.msg`, search the whole workspace for the field name, not
just this app:

```bash
grep -rn "controls_namespace" ~/nepi_engine_ws/src ~/first_robotics
```

`ControlsSandboxStatus` is app-local, so its consumers are few. A field in
`nepi_interfaces` may have dozens across every repo.

### 6.6 Display a list or repeated group of values

This app already does this, for all four array types. The pattern is worth
copying because the two array branches solve two different layout problems.

**Fixed-size widgets, laid out inline** — the `Bools` branch, lines 157-170:

```jsx
    if (type === "Bools") {
      const values = datum_msg.value_bools || []
      return (
        <Label title={display_name} key={name}>
          <div>
            {values.map((v, i) => (
              <div key={name + '_' + i} style={{ display: "inline-block", marginRight: Styles.vars.spacing.regular }}>
                <BooleanIndicator title={description} value={(v === true)} />
              </div>
            ))}
          </div>
        </Label>
      )
    }
```

Each indicator is a fixed size, so they are wrapped in `inline-block` divs with
a fixed gap and simply sit next to each other.

**Variable-width boxes, split across the row** — the `Strings`/`Ints`/`Floats`
branch, lines 187-206:

```jsx
    if (type === "Strings" || type === "Ints" || type === "Floats") {
      var values = []
      if (type === "Strings") { values = datum_msg.value_strings || [] }
      else if (type === "Ints") { values = datum_msg.value_ints || [] }
      else { values = (datum_msg.value_floats || []).map((v) => round(v, decimals)) }
      const boxWidth = (values.length > 0) ? Math.floor(90 / values.length) + "%" : "90%"
      return (
        <Label title={display_name} key={name}>
          {values.map((v, i) => (
            <Input
              key={name + '_' + i}
              disabled
              title={description}
              style={{ width: boxWidth, float: "left" }}
              value={v}
            />
          ))}
        </Label>
      )
    }
```

Three things here are worth understanding before you copy it:

- **`boxWidth` divides the row.** Two values get 45% each, three get 30% each.
  The 90 rather than 100 leaves margin. The `values.length > 0` guard exists
  because dividing by zero would produce `Infinity%`.
- **`.map()` inside `.map()`.** The `Floats` line rounds every element before
  display — `(datum_msg.value_floats || []).map((v) => round(v, decimals))` —
  and then the outer `.map()` turns each rounded value into an `Input`. Two
  passes over the same list, one to format and one to draw.
- **`|| []` on every array read.** A `DataStatus` that arrives before the node
  has populated a field gives `undefined`, and `.map()` on `undefined` throws,
  which blanks the entire panel. Never read an array field from a ROS message
  without this guard.

**To add a repeated readout of your own**, declare an array type and write a
list:

```python
        'motor_temps_data': {
            'type': 'Floats', 'value': [0.0, 0.0, 0.0, 0.0], 'round_value': -1, 'round_display': 1,
            'display_name': 'Motor Temps (C)', 'description': 'Per-motor temperature.', 'hidden': False},
```

```python
    self.data_if.set_datum_value('motor_temps_data', self.read_motor_temps())
```

The row draws itself, four boxes at 22% each. **The list length may change
between writes** — `boxWidth` is recomputed on every render, so a three-element
write after a four-element one simply redraws as three wider boxes. There is no
fixed size to declare anywhere.

There is no per-element label. If four unlabelled boxes in a row are not clear
enough, either put the ordering in the description — the tooltip is on every box
— or declare four separate `Float` data, each with its own `display_name`. Four
labelled rows beats one ambiguous row.

---

## 7. Build and test

**Which rebuild does my change need?**

| What you changed | What to run |
| --- | --- |
| A Python dict entry, a callback body, anything in `scripts/` | Deploy the script, restart the app. No RUI build. |
| Anything in `rui/*.js` | Full RUI build. |
| `RUI_DICT` in `params/*.yaml` | Full RUI build (the registration is generated at build time). |
| A field in `msg/*.msg` | Full catkin build, then restart. |

### Deploying a Python-only change

`deploy_app.sh` in this directory does both halves in one run: it rsyncs the
whole package into the build repo at
`/mnt/nepi_storage/nepi_src/nepi_engine_ws/src/nepi_apps/nepi_app_controls_sandbox`,
and then rsyncs just `scripts/` straight into the live install at
`/opt/nepi/nepi_engine/lib/nepi_app_controls_sandbox`.

It requires `NEPI_REMOTE_SETUP` to be set — the script exits with
`Must have environtment variable NEPI_REMOTE_SETUP set` if it is not:

```bash
cd /home/production/first_robotics/nepi_app_controls_sandbox

export NEPI_REMOTE_SETUP=1     # 1 = deploy to a remote NEPI device over ssh
./deploy_app.sh
```

Use `NEPI_REMOTE_SETUP=0` when you are running on the device itself; that path
skips ssh for the build-repo sync. The target device is `NEPI_TARGET_IP` at the
top of the script, and the ssh key is `~/.ssh/nepi_default_ssh_key`.

The live half of the deploy copies `scripts/` only. **A `rui/`, `params/` or
`msg/` change is not picked up by the live sync** — it reaches the build repo,
and takes effect on the next build.

Then restart the app so the node re-reads its dicts. From the RUI, disable and
re-enable the app on the Apps page; `apps_mgr` polls every 5 seconds and will
relaunch the node.

### Rebuilding the RUI

Run from the workspace root, not from this app:

```bash
cd /home/production/nepi_engine_ws
./build_nepi_rui.sh
```

That script does four things in order, and it is worth knowing which one is
which when it fails:

1. Rsyncs `src/nepi_rui/` into `${NEPI_BASE}/nepi_rui/`.
2. Overlays any system config from `${NEPI_CONFIG}/system_cfg/`.
3. **Regenerates the app registration.** It walks
   `${NEPI_RUI}/src/rui_webserver/rui-app/src/apps`, reads `rui_main_file` and
   `rui_main_class` out of every app's params YAML, and rewrites lines 27 and 31
   of `Nepi_IF_Apps.js` with the generated import and map lines. This is the
   step that makes hand-editing `Nepi_IF_Apps.js` pointless.
4. Runs `npm run build` in `src/rui_webserver/rui-app/`.

For a `rui/*.js` change to reach step 4 it must first reach the build repo,
which means running `deploy_app.sh` before `build_nepi_rui.sh`. The app's
`CMakeLists.txt` installs `rui/` flat into
`${NEPI_RUI}/src/rui_webserver/rui-app/src` and `params/` into
`.../src/apps`, so the catkin install is what puts your files where step 3 and
step 4 will find them.

Refresh the browser after the build finishes. The RUI is served on port 5003.

### Rebuilding after a `.msg` change

```bash
cd /home/production/nepi_engine_ws
./build_nepi_code.sh
```

Then restart the node. Everything that consumes the changed message needs this
build, not just this app — see 6.5.

### Checking it worked without opening the browser

The two status topics tell you whether the node side is right, which separates a
Python bug from a JavaScript bug in one command:

```bash
rostopic echo -n1 /nepi/<device>/app_controls_sandbox/controls/status
rostopic echo -n1 /nepi/<device>/app_controls_sandbox/data/status
```

Your new control should be in `controls_name_list` with its type in the matching
position of `controls_type_list`. Your new datum should be in `data_name_list`
with a live value in the matching `data_msg_list` entry.

If it is there and the RUI does not show it, the problem is in the RUI or the
build. If it is not there, the problem is in the Python and no amount of
rebuilding the RUI will help.

---

## 8. Troubleshooting

| Symptom | Likely cause | Check this first |
| --- | --- | --- |
| **Panel renders blank** — the box and its title are there, nothing inside | A JavaScript error while rendering. Most often `.map()` on a field that is `undefined` because a `.msg` field name changed or the message arrived unpopulated. | Browser console (F12). Then confirm every array read in your edit has the `\|\| []` guard, as at `NepiAppControlsSandbox-Data.js:243-246`. |
| **Panel blank, no error** — box drawn, no rows | No status message has arrived. Both boxes gate on `status_msg != null` (`-Controls.js:451`, `-Data.js:242`) and draw nothing until one does. | `rostopic echo -n1 .../controls/status`. If it hangs, the node is not running or `controls_namespace` is wrong. |
| **Panel missing entirely** — no box at all | The build did not pick the file up: a filename missing from `RUI_DICT.rui_files`, or a basename collision with another app, or a compile error that failed the build. | `params/controls_sandbox_app_params.yaml`, then the tail of the `build_nepi_rui.sh` output for a compile failure. |
| **Control shows but nothing happens** | Wrong topic or wrong message type on the wire — nothing is subscribed. `Trigger` always does this by design-defect. | `rostopic echo` the `set_*_control_value` topic while you click. Traffic and no effect means a type mismatch; no traffic means the send never fired — check for a missing `() =>`. |
| **Value shows as `undefined`** | Reading a message field that does not exist. Renamed `.msg` field, or a typo, or reading the wrong type's field (`value_int` on a `Float` datum). | The field list in `nepi_interfaces/msg/Datum.msg` or `Control.msg` against what your code reads. JavaScript will not warn you. |
| **Value shows as `NaN`** | Arithmetic on something that is not a number — usually a string from `round()` (which returns `toFixed()` output), or `parseInt` on empty text. | Where the value is computed. `round()` returns a string; multiplying it works by coercion, adding to it concatenates. |
| **Node errors on an unknown field** | Python is writing a datum or reading a control that is not declared. | That the key in `set_datum_value()` / `get_control_value()` exactly matches the dict key. Both fail quietly — a missing datum write is a no-op, a missing control read returns `None`, and the error surfaces later as `TypeError` on `None`. |
| **Control missing from the box, no error anywhere** | The entry raised while being built. `create_controls_dict()` wraps each entry in a bare `try/except: pass`. A bad `type` string, a `Float` default outside its bounds, or any `FloatSliders` control all vanish this way. | The `'type'` spelling against `CONTROL_TYPES`, then that the default is inside the bounds. |
| **Data row appears but never changes** | Declared but never written. | `dataUpdaterCb()` for a matching `set_datum_value()` call. |
| **Change appears one second late** | Expected. `ControlsIF.set_control_value()` writes bare `self.publish_status` with no parentheses, so the confirming status waits out the 1 Hz timer. | Nothing. Not a fault. |
| **Edits not appearing after a rebuild** | The change never reached the build. `deploy_app.sh`'s live sync copies `scripts/` only — `rui/`, `params/` and `msg/` changes go through the build repo. | Timestamp the installed file: `ls -l ${NEPI_RUI}/src/rui_webserver/rui-app/src/NepiAppControlsSandbox-Controls.js`. Then hard-refresh the browser; the bundle caches. |
| **A control you deleted is still there** | A stale saved config. `ControlsIF.init()` reads `/controls_controls_dict` back from the param server and overwrites the freshly built dict. | Factory-reset the node's config, then restart. |

---

## 9. Checklist before you call an edit done

- [ ] The dict key is unchanged, or every reference to it was updated together.
- [ ] A new control's `type` is spelled exactly as in `CONTROL_TYPES`; a new
      datum's, exactly as in `DATUM_TYPES`.
- [ ] A `Float` or `FloatSlider` default is inside its own bounds.
- [ ] A new datum is both declared in `createDataInitDict()` **and** written in
      `dataUpdaterCb()`.
- [ ] Nothing in `NepiAppControlsSandbox-Data.js` publishes.
- [ ] Any new array read from a ROS message has a `|| []` guard.
- [ ] Every new JSX callback is wrapped in `() =>`.
- [ ] No new file was added to `rui/` without being listed in
      `RUI_DICT.rui_files`, and its basename starts with
      `NepiAppControlsSandbox-`.
- [ ] `Nepi_IF_Apps.js` and `NepiApps.js` were not edited.
- [ ] `Nepi_IF_Controls.js` and `Nepi_IF_Data.js` in `nepi_rui` were not edited —
      they are shared, and `Nepi_IF_Controls.js` has four other consumers
      (`nepi_app_obstacles`, `nepi_app_auto_move`, `nepi_app_stereo_cam`,
      `nepi_app_wpilib_if`).
- [ ] A removed control or datum left no dead `get_control_value()` /
      `set_datum_value()` call behind — confirmed by grep.
- [ ] A shared render branch was not deleted along with the last item that used
      it.
- [ ] Any Python method you added is named to convention: public is
      `snake_case` and takes a docstring, private is `camelCase` — with or
      without a leading underscore — and takes none. A `Cb` suffix marks a ROS
      callback, and renaming one means auditing every call site that registers
      it first.
- [ ] The right rebuild was run for what changed (section 7), and
      `rostopic echo` on the status topic shows the change on the node side
      before you go looking at the browser.

---

## 10. What this app owns, and what it does not

`NepiAppControlsSandbox-Controls.js` and `NepiAppControlsSandbox-Data.js` are
**app-local forks** of `Nepi_IF_Controls.js` and `Nepi_IF_Data.js` in
`nepi_rui`. As of this writing the Controls fork differs from its origin only in
the header comment and the class name — it has not diverged yet. That is fine
and expected; the fork exists so it *can* diverge without touching the shared
component.

Change these two files freely. **Do not change the originals in `nepi_rui`.**

`NepiAppControlsSandbox-Settings.js` is a different case: it is a
sandbox-specific panel, not a fork, and most of what it publishes has no
subscriber (see the note in 5.1). Treat it as unfinished.

Everything below the app — `nepi_sdk/nepi_controls.py`, `nepi_sdk/nepi_data.py`,
`nepi_api/system_if.py`, `nepi_api/data_if.py`, and the message definitions in
`nepi_interfaces` — is shared platform code with other consumers. The rough
edges catalogued in the README and repeated through this guide live there. Work
around them in the app; fixing them is a platform change with its own review.
