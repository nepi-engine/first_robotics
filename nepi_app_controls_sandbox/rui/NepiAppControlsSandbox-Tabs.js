/*
#
# Copyright (c) 2024 Numurus <https://www.numurus.com>.
#
# This file is part of nepi rui (nepi_rui) repo
# (see https://github.com/nepi-engine/nepi_rui)
#
# License: NEPI RUI repo source-code and NEPI Images that use this source-code
# are licensed under the "Numurus Software License",
# which can be found at: <https://numurus.com/wp-content/uploads/Numurus-Software-License-Terms.pdf>
#
# Redistributions in source code must retain this top-level comment block.
# Plagiarizing this software to sidestep the license obligations is illegal.
#
# Contact Information:
# ====================
# - mailto:nepi@numurus.com
#
 */

/*
 * APP-LOCAL tab bar for nepi_app_controls_sandbox. No shared tab component
 * exists in nepi_rui (checked: no Tab*.js anywhere under
 * rui_webserver/rui-app/src/), so this is built from scratch using the same
 * Styles.js color tokens every other component in this app already uses
 * (grey1 borders, blue active-state, white background). Presentation only:
 * it holds which tab is active and renders whichever child corresponds to it.
 * It does not touch ROS, does not filter or transform its children's data,
 * and does not know what a "control" or "datum" is.
 *
 * Two rules keep it working, matching every other file in this directory:
 *   1. This file must stay listed under RUI_DICT.rui_files in
 *      params/controls_sandbox_app_params.yaml.
 *   2. Its basename must stay unique across all app packages (flat install).
 */
import React, { Component } from "react"

import Styles from "./Styles"

const styles = Styles.Create({
  // Matches Section.js's own bordered-box look (border: 1px solid grey1,
  // marginTop + padding regular) so the tab group reads as one panel, the
  // same way concept_3_tabbed_groups.html's .panel wraps its .tabbar.
  panel: {
    textAlign: "left",
    marginTop: Styles.vars.spacing.regular,
    padding: Styles.vars.spacing.regular,
    border: `1px solid ${Styles.vars.colors.grey1}`
  },
  bar: {
    display: "flex",
    borderBottom: `1px solid ${Styles.vars.colors.grey1}`,
    marginBottom: Styles.vars.spacing.regular
  },
  tab: {
    flex: 1,
    textAlign: "center",
    padding: `${Styles.vars.spacing.small.raw}px ${Styles.vars.spacing.xs.raw}px`,
    cursor: "pointer",
    fontSize: Styles.vars.fontSize.small,
    textTransform: "uppercase",
    letterSpacing: "0.03em",
    color: Styles.vars.colors.grey2,
    borderRight: `1px solid ${Styles.vars.colors.grey1}`
  },
  tabLast: {
    borderRight: "none"
  },
  tabActive: {
    background: Styles.vars.colors.blue,
    color: Styles.vars.colors.white,
    fontWeight: "bold"
  }
})

// Tab bar + tab pages. `tabs` is an array of { title, content }; only the
// active tab's content is rendered (the mockup's showTab() behavior, done
// with React state instead of direct DOM class toggling).
class NepiAppControlsSandboxTabs extends Component {
  constructor(props) {
    super(props)

    this.state = {
      activeIndex: 0
    }
  }

  render() {
    const tabs = this.props.tabs || []
    const activeIndex = this.state.activeIndex

    return (
      <div style={styles.panel}>
        <div style={styles.bar}>
          {tabs.map((tab, i) => {
            const tabStyle = {
              ...styles.tab,
              ...(i === tabs.length - 1 ? styles.tabLast : {}),
              ...(i === activeIndex ? styles.tabActive : {})
            }
            return (
              <div
                key={tab.title}
                style={tabStyle}
                onClick={() => this.setState({ activeIndex: i })}
              >
                {tab.title}
              </div>
            )
          })}
        </div>
        {(tabs[activeIndex] !== undefined) ? tabs[activeIndex].content : null}
      </div>
    )
  }
}

export default NepiAppControlsSandboxTabs
