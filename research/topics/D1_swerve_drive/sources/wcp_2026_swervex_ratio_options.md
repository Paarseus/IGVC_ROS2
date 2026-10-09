> For the complete documentation index, see [llms.txt](https://docs.wcproducts.com/welcome/llms.txt). Markdown versions of documentation pages are available by appending `.md` to page URLs; this page is available as [Markdown](https://docs.wcproducts.com/welcome/gearboxes/wcp-swerve-x/general-info/ratio-options.md).

# Ratio Options

## General Ratio Spread

The X gear ratios are designed to be a consistent spread across all Swerve X configs. **All X Gear Ratios come with 3 pinions on the same pitch to get the spread.**

{% hint style="info" %}
The Kraken X60 without FOC is used as a reference for the table below.
{% endhint %}

| Gear Ratio Set | Speed Spread (ft/s) |
| -------------- | ------------------- |
| X1             | \~13 to \~16        |
| X2             | \~15 to \~18        |
| X3             | \~18 to \~23        |

## Swerve X Gear Ratios (Standard)

{% hint style="info" %}
The Rotation Ratio is the same throughout all motor configurations using the 10t pinions, which is compatible with all motors. The overall rotation gear ratio will be **396/35:1** or **11.3142:1**
{% endhint %}

<div data-full-width="false"><figure><img src="https://3958382810-files.gitbook.io/~/files/v0/b/gitbook-x-prod.appspot.com/o/spaces%2FND1U7zTGCCQQKREZ5H8s%2Fuploads%2FfUe8XjKzoTjnoL5Z2uQB%2FX%20Gear%20Ratio%20Table%20-%20Standard.svg?alt=media&amp;token=87652c9d-bdf4-4d99-9d83-d3b43c26b394" alt=""><figcaption><p>Standard Drive Ratios</p></figcaption></figure></div>

## Swerve X Gear Ratios (Flipped, Gears Below)

{% hint style="info" %}
The Rotation Ratio is the same throughout all motor configurations using the 10t pinions, which is compatible with all motors. The overall rotation gear ratio will be **468/35:1** or **13.3714:1**
{% endhint %}

<figure><img src="https://3958382810-files.gitbook.io/~/files/v0/b/gitbook-x-prod.appspot.com/o/spaces%2FND1U7zTGCCQQKREZ5H8s%2Fuploads%2FQBLySEnBhQbdgV7CjQAj%2FX%20Gear%20Ratio%20Table%20-%20Flipped.svg?alt=media&amp;token=562d9738-c4b3-4602-bcf1-cc2d1ba51d68" alt=""><figcaption><p>(Flipped, Gears Below) Drive Ratios</p></figcaption></figure>


---

# Agent Instructions
This documentation is published with GitBook. GitBook is the documentation platform designed so that both humans and AI agents can read, navigate, and reason over technical content effectively. Learn more at gitbook.com.

## Querying This Documentation
If you need additional information that is not directly available in this page, you can query the documentation dynamically by asking a question.

Perform an HTTP GET request on the current page URL with the `ask` query parameter, and the optional `goal` query parameter:

```
GET https://docs.wcproducts.com/welcome/gearboxes/wcp-swerve-x/general-info/ratio-options.md?ask=<question>&goal=<endgoal>
```

`ask` is the immediate question: it should be specific, self-contained, and written in natural language.
`goal` is optional and describes the broader end goal you are ultimately trying to accomplish on behalf of the user. GitBook uses it to tailor the answer towards what is most useful for that goal.

The response will contain a direct answer to the question and relevant excerpts and sources from the documentation.

Use this mechanism when the answer is not explicitly present in the current page, you need clarification or additional context, or you want to retrieve related documentation sections.
