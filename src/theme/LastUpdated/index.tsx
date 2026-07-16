/**
 * Copyright (c) Facebook, Inc. and its affiliates.
 *
 * This source code is licensed under the MIT license found in the
 * LICENSE file in the root directory of this source tree.
 */

import React, {type ReactNode} from 'react';
import Translate from '@docusaurus/Translate';
import {ThemeClassNames} from '@docusaurus/theme-common';
import {useDateTimeFormat} from '@docusaurus/theme-common/internal';
import type {Props} from '@theme/LastUpdated';

function LastUpdatedAtDate({
  lastUpdatedAt,
}: {
  lastUpdatedAt: number;
}): ReactNode {
  const atDate = new Date(lastUpdatedAt);
  const dateTimeFormat = useDateTimeFormat({
    day: 'numeric',
    month: 'short',
    year: 'numeric',
    timeZone: 'UTC',
  });
  const formattedLastUpdatedAt = dateTimeFormat.format(atDate);

  return (
    <Translate
      id="theme.lastUpdated.atDate"
      description="The words used to describe on which date a page has been last updated"
      values={{
        date: (
          <b>
            <time dateTime={atDate.toISOString()} itemProp="dateModified">
              {formattedLastUpdatedAt}
            </time>
          </b>
        ),
      }}>
      {' on {date}'}
    </Translate>
  );
}

function LastUpdatedByUser({
  lastUpdatedBy,
}: {
  lastUpdatedBy: string;
}): ReactNode {
  return (
    <Translate
      id="theme.lastUpdated.byUser"
      description="The words used to describe by who the page has been last updated"
      values={{user: <b>{lastUpdatedBy}</b>}}>
      {' by {user}'}
    </Translate>
  );
}

export default function LastUpdated({
  lastUpdatedAt,
  lastUpdatedBy,
}: Props): ReactNode {
  const isDevelopment = process.env.NODE_ENV === 'development';
  const displayedLastUpdatedBy = isDevelopment && lastUpdatedBy
    ? '模拟作者'
    : lastUpdatedBy;

  return (
    <span className={ThemeClassNames.common.lastUpdated}>
      <Translate
        id="theme.lastUpdated.lastUpdatedAtBy"
        description="The sentence used to display when a page has been last updated, and by who"
        values={{
          atDate: lastUpdatedAt ? (
            <LastUpdatedAtDate lastUpdatedAt={lastUpdatedAt} />
          ) : (
            ''
          ),
          byUser: displayedLastUpdatedBy ? (
            <LastUpdatedByUser lastUpdatedBy={displayedLastUpdatedBy} />
          ) : (
            ''
          ),
        }}>
        {'Last updated{atDate}{byUser}'}
      </Translate>
      {isDevelopment && (
        <div>
          <small>
            <Translate
              id="theme.lastUpdated.devSimulationNotice"
              description="Notice that development preview uses simulated last-update metadata">
              {'（开发预览：作者与日期为模拟值）'}
            </Translate>
          </small>
        </div>
      )}
    </span>
  );
}
