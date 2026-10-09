> For the complete documentation index, see [llms.txt](https://docs.wcproducts.com/welcome/llms.txt). Markdown versions of documentation pages are available by appending `.md` to page URLs; this page is available as [Markdown](https://docs.wcproducts.com/welcome/gearboxes/wcp-swerve-x2/general-info/ratio-options.md).

# Ratio Options

## General Ratio Spread <a href="#general-ratio-spread" id="general-ratio-spread"></a>

The X gear ratios are designed to be a consistent spread across all Swerve X2 configs. **All X Gear Ratios come with 3 pinions on the same pitch to get the spread.**

{% hint style="info" %}
The Kraken X60 without FOC is used as a reference for the table below.
{% endhint %}

| Gear Ratio Set | Speed Spread (ft/s) |
| -------------- | ------------------- |
| X1             | \~13 to \~16        |
| X2             | \~15 to \~18        |
| X3             | \~16 to \~19        |
| X4             | \~18 to \~22        |

## Swerve X2 Gear Ratios <a href="#swerve-x-gear-ratios-standard" id="swerve-x-gear-ratios-standard"></a>

{% hint style="info" %}
The Rotation Ratio is the same throughout all motor configurations using the 10t pinions, which is compatible with all motors. The overall rotation gear ratio will be **12.1:1**
{% endhint %}

{% hint style="warning" %}
The 16t Second Stage Gear for the X2 Ratio Set is on a 18t Center Distance.

The 14t Second Stage Gear for the X4 Ratio Set is on a 16t Center Distance.
{% endhint %}

<figure><img src="https://4022448544-files.gitbook.io/~/files/v0/b/gitbook-x-prod.appspot.com/o/spaces%2Fv3U1blZmAL1arpmYDPqt%2Fuploads%2FPvLHrWvJMJuAK2A3ZfMQ%2FSwerve%20X2%20-%20Ratio%20Table.svg?alt=media&amp;token=fb33038d-f8d4-43a2-b680-e24942d97221" alt=""><figcaption></figcaption></figure>


---

# Agent Instructions
This documentation is published with GitBook. GitBook is the documentation platform designed so that both humans and AI agents can read, navigate, and reason over technical content effectively. Learn more at gitbook.com.

## Querying This Documentation
If you need additional information that is not directly available in this page, you can query the documentation dynamically by asking a question.

Perform an HTTP GET request on the current page URL with the `ask` query parameter, and the optional `goal` query parameter:

```
GET https://docs.wcproducts.com/welcome/gearboxes/wcp-swerve-x2/general-info/ratio-options.md?ask=<question>&goal=<endgoal>
```

`ask` is the immediate question: it should be specific, self-contained, and written in natural language.
`goal` is optional and describes the broader end goal you are ultimately trying to accomplish on behalf of the user. GitBook uses it to tailor the answer towards what is most useful for that goal.

The response will contain a direct answer to the question and relevant excerpts and sources from the documentation.

Use this mechanism when the answer is not explicitly present in the current page, you need clarification or additional context, or you want to retrieve related documentation sections.
